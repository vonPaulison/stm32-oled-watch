/**
 * ============================================================
 *  TempHum.c — 温湿度页面模块 (自治状态机 + 自绘显示)
 *
 *    1. 完全自治: 内部管理测量节奏/重绘/故障诊断,
 *       不依赖 watch.c 的私有变量
 *    2. 懒初始化: 首次进入本页才调 AHT20_Init()
 *       (AHT20_Init 内部自带 I2C_BusInit, 不依赖 OLED 的
 *        初始化顺序; 传感器测量完自动休眠 0.2uA, 退出页面
 *        无需像 QMC 那样做挂起操作)
 *    3. 刷新策略: 每 ~1.08s 一轮测量 (手册 2.3 节要求采集
 *       周期 >1s/次, 测太快传感器自热会引入误差),
 *       有新数据才重绘, 平时每 tick 只做一次忙位轮询
 *    4. 测量全非阻塞: 触发即回, 忙位每个 tick 轮询一次,
 *       就绪 (约80ms = 8个tick) 后读数, 不卡 10ms 主循环
 *
 *  总线使用: 走 System/I2C 通用模块 (I2C_ScanBus 诊断),
 *  传感器收发在 AHT20.c 驱动内部, 本页面不直接碰 I2C 时序
 * ============================================================
 */

#include "TempHum.h"
#include "watch.h"          /*NowState (extern), Watch_ExitLeafPage, TempHum 枚举值*/
#include "Key.h"            /*Key_IsPressed, KEY_xxx*/
#include "OLED.h"           /*OLED_xxx 绘图与文字*/
#include "I2C.h"            /*I2C_ScanBus — 模块不在线时的总线诊断*/
#include "AHT20.h"          /*温湿度传感器驱动*/

/*══════════════════════════════════════════════════════════════
 *  测量节奏参数 (tick = 10ms, 由 main 的 Delay_ms(10) 保证)
 *══════════════════════════════════════════════════════════════*/
/*两次测量触发间隔 100 tick = 1s; 加上 ~80ms 测量时间,
 *实际周期 ~1.08s, 满足手册 2.3 节"采集周期 >1s/次"的要求
 */
#define TH_IDLE_TICKS   100

/*等待数据超时 500ms: 正常测量只忙 ~80ms, 超时说明模块中途
 *被拔走等异常, 放弃本轮防状态机卡死在 BUSY
 */
#define TH_BUSY_TIMEOUT 50

/*测量状态机*/
#define TH_ST_IDLE      0       /*间隔等待中 (距下次触发)*/
#define TH_ST_BUSY      1       /*已触发, 等芯片测完 (忙位=1)*/

/*══════════════════════════════════════════════════════════════
 *  页面布局 (128x64) — 列位置按 8x16 字符宽 (每字符 8px) 算好:
 *    T/H 行 (y20~35 / y40~55):
 *      标签 x16~31 | 符号 x36~43 | 整数 x44~59 | 小数点 x60~67
 *      | 小数 x68~83 | 单位 x88 起
 *      两行整数/小数点列共用 → 小数点上下对齐
 *    占位符 "--.--" 5字符×8px=40px, x44~83, 与数值区等宽
 *    状态行 (6x8, y56~63): ST:xx 占 x4~35, 提示占 x76~123
 *══════════════════════════════════════════════════════════════*/
#define TH_ROW_T_Y      20      /*温度行 Y (8x16)*/
#define TH_ROW_H_Y      40      /*湿度行 Y (8x16)*/
#define TH_COL_LABEL    16      /*"T:"/"H:" 标签列*/
#define TH_COL_SIGN     36      /*温度符号列 (湿度行此位留空)*/
#define TH_COL_VALUE    44      /*整数部分/占位符起始列 (两行共用)*/
#define TH_COL_DOT      60      /*小数点列 (两行共用 = 小数点对齐)*/
#define TH_COL_FRAC     68      /*小数部分列*/
#define TH_COL_UNIT     88      /*单位区起始列 (°C 的圆环 / %)*/
#define TH_STATUS_Y     56      /*底部状态行 Y (6x8)*/

/*══════════════════════════════════════════════════════════════
 *  私有状态变量 (仅本文件可见, static ——"能藏就藏")
 *══════════════════════════════════════════════════════════════*/
static uint8_t  th_first_enter = 1;     /*首次进入标志 (触发传感器初始化)*/
static uint8_t  th_sensor_ok   = 0;     /*传感器在线标志 (Init 结果)*/
static uint8_t  th_redraw      = 1;     /*重绘标志*/

static uint8_t  th_state       = TH_ST_IDLE;
static uint16_t th_wait_cnt    = 0;     /*IDLE=距下次触发的tick数 / BUSY=已等时长*/

static uint8_t  th_has_data    = 0;     /*收到过至少一帧有效数据? (控制占位符)*/
static int32_t  th_temp_c100   = 0;     /*最新温度 ×100 (驱动定点输出, 可为负)*/
static uint32_t th_humi_rh100  = 0;     /*最新湿度 ×100*/
static uint8_t  th_crc_err     = 0;     /*最近一轮 CRC 校验失败 (状态行闪烁提示)*/
static uint8_t  th_status      = 0;     /*最近读到的状态字 (底部诊断行)*/

/*提示闪烁 (0.5秒翻转一次, 同 Countdown/Compass 的套路)*/
static uint8_t  th_blink       = 1;
static uint8_t  th_blink_cnt   = 0;

/*I2C 诊断 (模块不在线时用): 扫描结果 + 重扫节拍
 *th_found 里应能看到 0x3C (OLED) —— 连它都没有 = 驱动问题,
 *有 0x3C 没 0x38 = 接线/芯片问题, 有 0x39 = 地址引脚变体
 */
static uint8_t  th_scan_cnt    = 0;      /*重扫节拍计数 (50 tick = 0.5s)*/
static uint8_t  th_found[4]    = {0, 0, 0, 0};
static uint8_t  th_scan_n      = 0;      /*扫描到的设备数*/

/**
 * 查地址是否在最近一次扫描结果里 (与 Compass 同款)
 */
static uint8_t TempHum_AddrFound(uint8_t addr)
{
    uint8_t i;
    for (i = 0; i < th_scan_n; i++)
    {
        if (th_found[i] == addr)
            return 1;
    }
    return 0;
}

/**
 * 温湿度页面绘制 (全屏: 清屏→画全部→Update)
 */
static void TempHum_Display(void)
{
    uint32_t t_abs;                 /*温度定点值的绝对值*/

    th_redraw = 0;
    OLED_Clear();

    /*── 标题 "TempHum" 居中 (7字符×8px=56, (128-56)/2=36) ──*/
    OLED_ShowString(36, 0, "TempHum", OLED_8X16);
    OLED_ReverseArea(0, 0, 128, 16);

    /*── 温度行: T: +25.37 °C ──
     *符号/整数/小数手工拼, 不用 ShowSignedNum:
     *整数部分 = 定点值/100 会向零截断, -0.05℃ (c100值=-5)
     *截断成 0 → 按整数部分画符号会错成 "+00.05"。
     *符号必须按原始定点值的正负画, 数值取绝对值再拆
     */
    OLED_ShowString(TH_COL_LABEL, TH_ROW_T_Y, "T:", OLED_8X16);
    if (th_has_data)
    {
        t_abs = (th_temp_c100 < 0) ? (uint32_t)(-th_temp_c100)
                                   : (uint32_t)th_temp_c100;
        OLED_ShowChar(TH_COL_SIGN, TH_ROW_T_Y,
                      (th_temp_c100 < 0) ? '-' : '+', OLED_8X16);
        OLED_ShowNum(TH_COL_VALUE, TH_ROW_T_Y, t_abs / 100, 2, OLED_8X16);
        OLED_ShowChar(TH_COL_DOT, TH_ROW_T_Y, '.', OLED_8X16);
        OLED_ShowNum(TH_COL_FRAC, TH_ROW_T_Y, t_abs % 100, 2, OLED_8X16);
    }
    else
    {
        /*首帧还没测完: 占位符, 宽度与数值区一致 (x44~83)*/
        OLED_ShowString(TH_COL_VALUE, TH_ROW_T_Y, "--.--", OLED_8X16);
    }
    /*度符号: 2px 小圆环当 ° (ASCII 字模里没有这个字符),
     *圆心 y=行顶+4, 视觉上是上标; 后面跟 "C"
     */
    OLED_DrawCircle(TH_COL_UNIT, TH_ROW_T_Y + 4, 2, OLED_UNFILLED);
    OLED_ShowString(TH_COL_UNIT + 6, TH_ROW_T_Y, "C", OLED_8X16);

    /*── 湿度行: H: 45.60 % (无符号, 列与温度行对齐) ──*/
    OLED_ShowString(TH_COL_LABEL, TH_ROW_H_Y, "H:", OLED_8X16);
    if (th_has_data)
    {
        OLED_ShowNum(TH_COL_VALUE, TH_ROW_H_Y, th_humi_rh100 / 100, 2, OLED_8X16);
        OLED_ShowChar(TH_COL_DOT, TH_ROW_H_Y, '.', OLED_8X16);
        OLED_ShowNum(TH_COL_FRAC, TH_ROW_H_Y, th_humi_rh100 % 100, 2, OLED_8X16);
    }
    else
    {
        OLED_ShowString(TH_COL_VALUE, TH_ROW_H_Y, "--.--", OLED_8X16);
    }
    OLED_ShowString(TH_COL_UNIT, TH_ROW_H_Y, "%", OLED_8X16);

    /*── 底部状态行: 状态字原始值 (诊断) + CRC 错误闪烁提示 ──
     *健康值常见 0x10/0x18 (bit4 CRC标志=1, bit3 校准=1);
     *bit7(忙)=1 只会持续 ~80ms, 不会长期停在这个状态
     */
    OLED_ShowString(4, TH_STATUS_Y, "ST:", OLED_6X8);
    OLED_ShowHexNum(24, TH_STATUS_Y, th_status, 2, OLED_6X8);
    if (th_crc_err && th_blink)
        OLED_ShowString(76, TH_STATUS_Y, "CRC ERR!", OLED_6X8);

    OLED_Update();
}

/**
 * 模块未接入时的诊断画面 (与 Compass 的诊断页同套路)
 * 扫描整条 I2C 总线并把结果画出来, 按结果给出最可能的原因:
 *   0x3C = OLED (它正在显示本页, 必然在)
 *   0x38 = AHT20 (本驱动期望的芯片)
 *   0x39 = AHT20 地址引脚接高时的变体地址 (驱动固定用 0x38)
 */
static void TempHum_DisplayNoSensor(void)
{
    uint8_t i;

    th_redraw = 0;
    OLED_Clear();
    OLED_ShowString(36, 0, "TempHum", OLED_8X16);
    OLED_ReverseArea(0, 0, 128, 16);
    OLED_ShowString(28, 22, "NO SENSOR", OLED_8X16);    /*9字符×8=72, x28*/

    /*── 扫描结果行: 总线上应答的地址 (16进制, 每2字符占24px) ──*/
    OLED_ShowString(4, 42, "SCAN:", OLED_6X8);
    if (th_scan_n == 0)
    {
        OLED_ShowString(36, 42, "NONE", OLED_6X8);
    }
    else
    {
        for (i = 0; i < th_scan_n; i++)
            OLED_ShowHexNum(36 + i * 24, 42, th_found[i], 2, OLED_6X8);
    }

    /*── 提示行: 按扫描结果定位问题 ──*/
    if (th_scan_n == 0)
        OLED_ShowString(4, 52, "I2C ERR, report", OLED_6X8);
    else if (TempHum_AddrFound(0x38))
    {
        /*0x38 有应答但初始化一直失败 → 读状态字判断芯片反应
         *(0xFF = 连状态字都读不出, 总线时好时坏)
         */
        uint8_t st = 0xFF;
        AHT20_ReadStatus(&st);
        OLED_ShowString(4, 52, "38 ST:", OLED_6X8);
        OLED_ShowHexNum(40, 52, st, 2, OLED_6X8);
    }
    else if (TempHum_AddrFound(0x39))
        OLED_ShowString(4, 52, "chip @39, not 38", OLED_6X8);
    else
        OLED_ShowString(4, 52, "chk SDA/SCL/VCC", OLED_6X8);

    OLED_Update();
}

/**
 * 温湿度页面主节拍 — 由 Watch_Tick 在 TempHum 页面调用
 */
void TempHum_Tick(void)
{
    uint8_t  ret;                   /*AHT20_ReadResult 返回值*/
    int32_t  t100;                  /*本轮温度 ×100*/
    uint32_t h100;                  /*本轮湿度 ×100*/

    /*══ 首次进入: 初始化传感器 (懒初始化) ══
     *AHT20_Init 内部先调 I2C_BusInit (配引脚+总线卡死恢复),
     *不依赖 OLED_Init 的顺序; 模块没接也会安全返回 0
     */
    if (th_first_enter)
    {
        th_first_enter = 0;
        th_sensor_ok   = AHT20_Init();
        if (!th_sensor_ok)
        {
            /*初始化失败: 先扫一遍总线再画诊断页,
             *首帧就带扫描结果, 不用等 0.5s 重扫周期
             */
            th_scan_n = I2C_ScanBus(th_found, 4);
            TempHum_DisplayNoSensor();
            return;
        }
        /*就绪: 顺手读一次状态字给底部诊断行, 并立刻触发
         *首轮测量 (wait_cnt 拉满 = 下个 tick 就触发, 不等 1s)
         */
        AHT20_ReadStatus(&th_status);
        th_state    = TH_ST_IDLE;
        th_wait_cnt = TH_IDLE_TICKS;
        th_redraw   = 1;
    }

    /*══ 防御: 非温湿度页面不执行 (与其他叶子模块同款) ══*/
    if (NowState != TempHum)
        return;

    /*══ 模块不在线: 诊断态 — 重扫总线/重试初始化/退出 ══*/
    if (!th_sensor_ok)
    {
        if (Key_IsPressed(KEY_BACK))
        {
            th_first_enter = 1;         /*下次进入重新探测*/
            Watch_ExitLeafPage();
            return;
        }

        /*0.5s 自动重扫一次; 按 OK 可以立即重扫
         *(插拔线/重焊接后不用退页重进, 盯着 SCAN 行变化即可)
         */
        th_scan_cnt++;
        if (th_scan_cnt < 50 && !Key_IsPressed(KEY_OK))
            return;                     /*未到重扫时间, 本tick结束*/

        th_scan_cnt = 0;
        th_scan_n = I2C_ScanBus(th_found, 4);

        /*只有看到 0x38 (本驱动期望地址) 才重试初始化;
         *0x39 是地址引脚变体, 驱动固定用 0x38, 重试也没用
         */
        if (TempHum_AddrFound(0x38))
            th_sensor_ok = AHT20_Init();

        if (!th_sensor_ok)
        {
            TempHum_DisplayNoSensor();  /*刷新诊断画面*/
            return;
        }
        th_state    = TH_ST_IDLE;
        th_wait_cnt = TH_IDLE_TICKS;    /*恢复在线: 立刻开测*/
        th_redraw   = 1;                /*本tick继续走正常流程*/
    }

    /*══ 测量状态机 (全程非阻塞) ══*/
    if (th_state == TH_ST_IDLE)
    {
        /*间隔计时到 → 触发新一轮测量*/
        th_wait_cnt++;
        if (th_wait_cnt >= TH_IDLE_TICKS)
        {
            if (AHT20_StartMeasure())
            {
                th_state    = TH_ST_BUSY;
                th_wait_cnt = 0;
            }
            else
            {
                /*触发时 NACK: 模块被拔走了 → 转诊断态*/
                th_sensor_ok = 0;
                th_scan_cnt  = 0;
                th_scan_n    = I2C_ScanBus(th_found, 4);
                TempHum_DisplayNoSensor();
                return;
            }
        }
    }
    else
    {
        /*TH_ST_BUSY: 轮询测量结果 (正常约 80ms = 8 个tick)*/
        ret = AHT20_ReadResult(&t100, &h100);
        if (ret == 1)
        {
            /*── 新数据到: 存值 + 切回间隔等待 ──*/
            th_temp_c100  = t100;
            th_humi_rh100 = h100;
            th_has_data   = 1;
            th_crc_err    = 0;
            th_state      = TH_ST_IDLE;
            th_wait_cnt   = 0;
            AHT20_ReadStatus(&th_status);   /*顺手刷新诊断行*/
            th_redraw     = 1;
        }
        else if (ret == 2)
        {
            /*── CRC 错: 本轮数据不可信已丢弃, 保留旧值显示,
             *状态行闪烁提示, 下一轮正常会自动恢复
             */
            th_crc_err    = 1;
            th_state      = TH_ST_IDLE;
            th_wait_cnt   = 0;
            AHT20_ReadStatus(&th_status);
            th_redraw     = 1;
        }
        else
        {
            /*── 还没测完 (或 NACK): 计时, 超时放弃本轮 ──
             *正常忙 ~80ms; 超时说明模块中途被拔走等异常,
             *回 IDLE, 下一轮触发时的 NACK 会转诊断态
             */
            th_wait_cnt++;
            if (th_wait_cnt >= TH_BUSY_TIMEOUT)
            {
                th_state    = TH_ST_IDLE;
                th_wait_cnt = 0;
            }
        }
    }

    /*══ 按键处理: 本页只用 BACK 退出 ══
     *AHT20 测完自动休眠 (0.2uA), 退出无需挂起操作;
     *UP/DOWN/OK 无功能, 残留 flag 由 watch.c 在状态切换时统一清
     */
    if (Key_IsPressed(KEY_BACK))
    {
        th_first_enter = 1;             /*下次进入重新初始化/探测*/
        Watch_ExitLeafPage();
        return;                         /*NowState已变, 不再绘制*/
    }

    /*══ CRC 提示闪烁定时 (0.5秒翻转, 同其他模块套路) ══
     *只在有 CRC 错误时才因闪烁重绘, 平时不受闪烁干扰
     */
    if (th_crc_err)
    {
        if (th_blink_cnt < 50)
            th_blink_cnt++;
        else
        {
            th_blink = !th_blink;
            th_blink_cnt = 0;
            th_redraw = 1;
        }
    }
    else
    {
        th_blink = 1;                   /*无错误时强制亮, 防止停在灭半周*/
        th_blink_cnt = 0;
    }

    /*══ 重绘 ══*/
    if (th_redraw)
        TempHum_Display();
}
