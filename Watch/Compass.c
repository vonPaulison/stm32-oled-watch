/**
 * ============================================================
 *  Compass.c — 指南针页面模块 (自治状态机 + 自绘显示)
 *
 *  设计要点 (与 Stopwatch/Countdown 同一套架构约定):
 *    1. 完全自治: 内部管理重绘/校准/传感器电源,
 *       不依赖 watch.c 的私有变量
 *    2. 懒初始化: 首次进入本页才调 QMC5883P_Init()
 *       (依赖 OLED_Init 已配好 I2C 引脚; 平时传感器挂起省电,
 *       只有停在本页才以约 78uA 运行, 手册 Table 2)
 *    3. 刷新策略: 传感器 10Hz 出新数据才重绘 (每100ms一次),
 *       平时每个 tick 只做一次状态寄存器轮询, 开销极小
 *    4. 退出即挂起: BACK 离开时 QMC5883P_Suspend(),
 *       下次进入重新初始化 (顺带支持模块热插拔重检测)
 *
 *  航向解算原理:
 *    地磁水平分量指向磁北。芯片平放时, X/Y 轴测得水平分量:
 *        heading = atan2f(东向分量, 北向分量)  → 0°=北, 顺时针增大
 *    默认假设模块 X 轴指向手表12点方向、Y 轴指向3点方向、
 *    芯片正面朝上。如果你的焊接/安装方向不同, 改下面的
 *    CP_MOUNT_* 三个宏即可 (交换轴 = 交换引用, 取反 = 加负号)
 *
 *  硬磁校准 (长按OK触发):
 *    手表在桌上/身上会被铁磁件磁化, 导致原始数据整体偏移,
 *    画出来不是圆心在原点的圆 → 指北方向有固定偏差。
 *    校准 = 水平缓慢旋转一整圈, 记录 X/Y 的 min/max,
 *    偏移 = (min+max)/2, 之后每次读数先减偏移再解算。
 *    (软磁畸变/椭球校准是进阶话题, 本版只做硬磁, 够日常用)
 *    偏移存 RAM, 断电丢失; 持久化可后续用 BKP/Flash (同亮度设置)
 * ============================================================
 */

#include <math.h>           /*atan2f/sinf/cosf — 航向角与指针解算*/
#include <string.h>         /*strlen — 方位名宽度计算*/
#include "Compass.h"
#include "watch.h"          /*NowState (extern), Watch_ExitLeafPage, Compass 枚举值*/
#include "Key.h"            /*Key_IsPressed/Key_IsLongPress, KEY_xxx*/
#include "OLED.h"           /*OLED_xxx 绘图与文字*/
#include "QMC5883P.h"       /*磁传感器驱动*/

/*══════════════════════════════════════════════════════════════
 *  安装方向修正 (按实际焊接方向改这里)
 *  默认假设: X 轴指向12点方向(前), Y 轴指向3点方向(右), 正面朝上
 *  例: 模块转了90°焊 → 改成 (cp_rawy) 和 (-cp_rawx)
 *  例: 模块倒着贴 → 前轴加负号
 *════════════════════════════════════════════════════════════*/
#define CP_MOUNT_FWD(x, y)  (x)     /*参与解算的"前方"分量 (12点方向)*/
#define CP_MOUNT_RIGHT(x, y) (y)    /*参与解算的"右方"分量 (3点方向)*/

/*磁偏角 (磁北 vs 真北): 中国东部约 -7°~-11°, 默认 0=指磁北
 *需要真北的话把当地磁偏角填进来 (东为正/西为负)
 */
#define CP_DECLINATION_DEG  0.0f

/*校准有效性阈值: 旋转一圈 X 或 Y 的摆幅 (max-min) 至少要这么大
 *地磁场约 0.5 Gauss = ±8G 量程下约 1875 LSB, 水平分量摆幅
 *正常应有几百到三千 LSB; 太小说明没认真转, 结果不可信
 */
#define CP_CAL_MIN_SPAN     500

/*══════════════════════════════════════════════════════════════
 *  罗盘盘面布局参数 (128x64)
 *    标题栏 0~15 (反相), 盘面圆心在左半区, 右半区放数字
 *════════════════════════════════════════════════════════════*/
#define CP_ROSE_CX      30          /*盘面圆心 X*/
#define CP_ROSE_CY      39          /*盘面圆心 Y (标题下居中偏下)*/
#define CP_ROSE_R       14          /*盘面半径*/

#define CP_CARDINAL_X   82          /*右半区八方位大字中心 X*/
#define CP_CARDINAL_Y   20          /*八方位大字 Y (8x16)*/
#define CP_HDG_X        72          /*角度数字 X (3位数字宽24px)*/
#define CP_HDG_Y        40          /*角度数字 Y (8x16)*/
#define CP_RAW_Y        56          /*底部三轴原始值行 Y (6x8)*/

/*══════════════════════════════════════════════════════════════
 *  私有状态变量 (仅本文件可见, static ——"能藏就藏")
 *════════════════════════════════════════════════════════════*/
static uint8_t  cp_first_enter = 1;     /*首次进入标志 (触发传感器初始化)*/
static uint8_t  cp_sensor_ok   = 0;     /*传感器在线标志 (Init 结果)*/
static uint8_t  cp_redraw      = 1;     /*重绘标志*/

static int16_t  cp_rawx = 0;            /*最新一组的原始三轴值 (未减偏移)*/
static int16_t  cp_rawy = 0;
static int16_t  cp_rawz = 0;

static uint16_t cp_heading_deg = 0;     /*航向角 0~359 (0=北, 顺时针)*/
static uint8_t  cp_cardinal    = 0;     /*八方位索引 0~7 (N/NE/E/...)*/

/*硬磁偏移 (校准结果, 减去后数据圆心才在原点)
 *注意跨页面保留 (物理属性, 不随进出页面丢失), 断电丢失
 */
static int16_t  cp_cal_offx    = 0;
static int16_t  cp_cal_offy    = 0;

/*校准状态: 0=正常显示, 1=采集中 (旋转手表)*/
static uint8_t  cp_cal_state   = 0;
static int16_t  cp_minx = 0;            /*采集中的 X/Y 极值 (用原始值统计)*/
static int16_t  cp_maxx = 0;
static int16_t  cp_miny = 0;
static int16_t  cp_maxy = 0;
static uint8_t  cp_cal_result  = 0;     /*校准结果提示: 0=无 1=成功 2=摆幅不足*/

/*校准提示闪烁 (0.5秒翻转一次, 同 Countdown 的 cd_blink 套路)*/
static uint8_t  cp_blink       = 1;
static uint8_t  cp_blink_cnt   = 0;

/*I2C 诊断 (模块不在线时用): 扫描结果 + 重扫节拍
 *cp_found 里应能看到 0x3C (OLED) —— 连它都没有 = 驱动问题,
 *有 0x3C 没 0x2C = 接线/芯片问题, 有 0x0D = 买到 L 版芯片
 */
static uint8_t  cp_scan_cnt    = 0;      /*重扫节拍计数 (50 tick = 0.5s)*/
static uint8_t  cp_found[4]    = {0, 0, 0, 0};
static uint8_t  cp_scan_n      = 0;      /*扫描到的设备数*/

/*八方位名称表: 索引 = (角度+22)/45, 0=北 顺时针
 *不加 const: OLED_ShowString 的参数是 char* (江协库签名), 传 const 会告警
 */
static char *cp_dir_name[8] = {
    "N", "NE", "E", "SE", "S", "SW", "W", "NW"
};

/**
 * 计算字符串像素宽度 (8x16 字体每字符 8px, 用于右半区居中)
 */
static uint8_t Compass_StrWidth8(char *s)
{
    return (uint8_t)(strlen(s) * 8);
}

/**
 * 把航向角换算成八方位索引
 * @param deg  航向角 0~359
 * @return 0~7 (N/NE/E/SE/S/SW/W/NW, 顺时针)
 * 公式: 每个扇区 45°, 加 22 让扇区以正北为中心对称
 *   例: 337°~22° → N,  23°~67° → NE
 */
static uint8_t Compass_CardinalIndex(uint16_t deg)
{
    return (uint8_t)(((deg + 22) % 360) / 45);
}

/**
 * 指南针页面绘制 (全屏: 清屏→画全部→Update)
 * 校准中和正常显示共用本函数, 按 cp_cal_state 切换底部提示
 */
static void Compass_Display(void)
{
    int16_t tipx, tipy;             /*指针端点*/
    int16_t tailx, taily;           /*指针反向短尾端点*/
    float   rad;                    /*航向角弧度*/
    float   fwd, right;             /*减偏移后的前/右分量*/
    uint8_t i;

    cp_redraw = 0;
    OLED_Clear();

    /*── 标题 "Compass" 居中 (7字符×8px=56, (128-56)/2=36) ──*/
    OLED_ShowString(36, 0, "Compass", OLED_8X16);
    OLED_ReverseArea(0, 0, 128, 16);

    /*── 盘面: 外圈圆 + 四方位字母 ──*/
    OLED_DrawCircle(CP_ROSE_CX, CP_ROSE_CY, CP_ROSE_R, OLED_UNFILLED);
    /*字母贴着圆环内侧放: N上/E右/S下/W左, 6x8 字体*/
    OLED_ShowChar(CP_ROSE_CX - 3,              CP_ROSE_CY - CP_ROSE_R + 2, 'N', OLED_6X8);
    OLED_ShowChar(CP_ROSE_CX + CP_ROSE_R - 5,  CP_ROSE_CY - 3,             'E', OLED_6X8);
    OLED_ShowChar(CP_ROSE_CX - 3,              CP_ROSE_CY + CP_ROSE_R - 9, 'S', OLED_6X8);
    OLED_ShowChar(CP_ROSE_CX - CP_ROSE_R + 1,  CP_ROSE_CY - 3,             'W', OLED_6X8);

    /*── 指针: 从圆心指向航向方向 ──
     *屏幕上方=北, 航向顺时针增大 → 端点公式:
     *  tipx = cx + r*sin(heading),  tipy = cy - r*cos(heading)
     *再画一小段反向尾巴, 指针更醒目 (类似真实磁针的红白两端)
     */
    /*注意: cp_heading_deg 解算时已含磁偏角修正, 这里只做角度→弧度*/
    rad = (float)cp_heading_deg * 3.14159265f / 180.0f;
    tipx = CP_ROSE_CX + (int16_t)(CP_ROSE_R * sinf(rad));
    tipy = CP_ROSE_CY - (int16_t)(CP_ROSE_R * cosf(rad));
    tailx = CP_ROSE_CX - (int16_t)(CP_ROSE_R / 2 * sinf(rad));
    taily = CP_ROSE_CY + (int16_t)(CP_ROSE_R / 2 * cosf(rad));
    OLED_DrawLine(CP_ROSE_CX, CP_ROSE_CY, tipx, tipy);      /*指向北的长针*/
    OLED_DrawLine(CP_ROSE_CX, CP_ROSE_CY, tailx, taily);    /*反向短尾*/

    /*── 右半区: 八方位大字 (居中于 CP_CARDINAL_X) ──*/
    i = Compass_StrWidth8(cp_dir_name[cp_cardinal]);
    OLED_ShowString(CP_CARDINAL_X - i / 2, CP_CARDINAL_Y, cp_dir_name[cp_cardinal], OLED_8X16);

    /*── 右半区: 航向角 (3位数字补零, 如 045/237) ──*/
    OLED_ShowNum(CP_HDG_X, CP_HDG_Y, cp_heading_deg, 3, OLED_8X16);
    OLED_ShowString(CP_HDG_X + 26, CP_HDG_Y + 8, "deg", OLED_6X8);

    /*── 底部行: 正常=三轴原始值, 校准=操作提示 ──*/
    if (cp_cal_state == 0)
    {
        /*原始值行 (校准前调试用)
         *ShowSignedNum 渲染宽度 = 1符号 + Length位数字
         *Length=4 → 每项 5字符×6px=30px:  X@10~40 Y@50~80 Z@90~120
         *超±9999的极端值只显示低4位 (贴磁铁才有, 调试行可接受)
         */
        OLED_ShowString(4, CP_RAW_Y, "X", OLED_6X8);
        OLED_ShowSignedNum(10, CP_RAW_Y, cp_rawx, 4, OLED_6X8);
        OLED_ShowString(44, CP_RAW_Y, "Y", OLED_6X8);
        OLED_ShowSignedNum(50, CP_RAW_Y, cp_rawy, 4, OLED_6X8);
        OLED_ShowString(84, CP_RAW_Y, "Z", OLED_6X8);
        OLED_ShowSignedNum(90, CP_RAW_Y, cp_rawz, 4, OLED_6X8);
        /*上一次校准失败的结果提示
         *放在标题正下方 (y=17): 避开罗盘盘面(最低到y53)和右侧航向数字(y40起)
         *旧位置 y=48 会和它们重叠 —— 字符打架
         */
        if (cp_cal_result == 2)
            OLED_ShowString(4, 17, "TOO FAST!", OLED_6X8);
    }
    else
    {
        /*校准中: 右侧整块让给校准进度显示
         *(旋转中航向数字乱跳没有意义, 正好腾出 y20~53 的右侧区域)
         *旧版把 SPN/TGT 放在 y=48, 和航向数字(8x16占y40~55)、
         *deg 字样(y48~55)横纵都撞车 —— 就是"字符重叠"的来源
         */
        /*当前摆幅 = X/Y 两轴摆幅的较大者
         *极值初值是 min=32767 / max=-32768, 还没采到样本时
         *直接相减会得到负数, 负数一律按 0 显示
         */
        int spanx = cp_maxx - cp_minx;
        int spany = cp_maxy - cp_miny;
        int span = (spanx > spany) ? spanx : spany;
        if (span < 0)
            span = 0;

        OLED_ShowString(66, 20, "SPAN", OLED_6X8);                  /*y20~27*/
        OLED_ShowNum(66, 28, (uint32_t)span, 5, OLED_8X16);         /*y28~43, 5位×8px=66~106*/
        OLED_ShowString(66, 46, "TGT", OLED_6X8);                   /*y46~53*/
        OLED_ShowNum(88, 46, CP_CAL_MIN_SPAN, 3, OLED_6X8);         /*y46~53*/

        /*提示语闪烁 (灭半周不画, 亮半周画)*/
        if (cp_blink)
            OLED_ShowString(4, CP_RAW_Y, "ROTATE 360! OK=END", OLED_6X8);
    }

    OLED_Update();
}

/**
 * 查地址是否在最近一次扫描结果里
 */
static uint8_t Compass_AddrFound(uint8_t addr)
{
    uint8_t i;
    for (i = 0; i < cp_scan_n; i++)
    {
        if (cp_found[i] == addr)
            return 1;
    }
    return 0;
}

/**
 * 模块未接入时的诊断画面
 * 扫描整条 I2C 总线并把结果画出来, 按结果给出最可能的原因:
 *   0x3C = OLED (它正在显示本页, 必然在)
 *   0x2C = QMC5883P (本驱动期望的芯片)
 *   0x0D = QMC5883L (市面老版本芯片, 寄存器/地址全不同)
 */
static void Compass_DisplayNoSensor(void)
{
    uint8_t i;

    cp_redraw = 0;
    OLED_Clear();
    OLED_ShowString(36, 0, "Compass", OLED_8X16);
    OLED_ReverseArea(0, 0, 128, 16);
    OLED_ShowString(28, 22, "NO SENSOR", OLED_8X16);

    /*── 扫描结果行: 总线上应答的地址 (16进制, 每2字符占24px) ──*/
    OLED_ShowString(4, 42, "SCAN:", OLED_6X8);
    if (cp_scan_n == 0)
    {
        OLED_ShowString(36, 42, "NONE", OLED_6X8);
    }
    else
    {
        for (i = 0; i < cp_scan_n; i++)
            OLED_ShowHexNum(36 + i * 24, 42, cp_found[i], 2, OLED_6X8);
    }

    /*── 提示行: 按扫描结果定位问题 (16字符×6px=96px 以内) ──*/
    if (cp_scan_n == 0)
        OLED_ShowString(4, 52, "I2C ERR, report", OLED_6X8);   /*连OLED都扫不到=驱动问题*/
    else if (Compass_AddrFound(0x2C))
    {
        /*0x2C 有应答但 ID 不是 0x80 → 显示真实 ID 判断芯片型号*/
        OLED_ShowString(4, 52, "2C ID:", OLED_6X8);
        OLED_ShowHexNum(40, 52, QMC5883P_ReadID(), 2, OLED_6X8);
    }
    else if (Compass_AddrFound(0x0D))
        OLED_ShowString(4, 52, "5883L chip @0D", OLED_6X8);    /*买到L版了*/
    else
        OLED_ShowString(4, 52, "chk SDA/SCL/VCC", OLED_6X8);   /*总线正常但没QMC=接线*/

    OLED_Update();
}

/**
 * 开始一次硬磁校准
 * 极值用"原始值"统计 (减偏移前的), 否则旧偏移会污染新统计
 */
static void Compass_CalStart(void)
{
    cp_cal_state  = 1;
    cp_minx = 32767;        /*极值初值: min 取最大, max 取最小*/
    cp_maxx = -32768;
    cp_miny = 32767;
    cp_maxy = -32768;
    cp_cal_result = 0;
    cp_blink = 1;           /*提示从"亮"开始*/
    cp_blink_cnt = 0;
    cp_redraw = 1;
}

/**
 * 结束校准: 算偏移, 判断摆幅是否够
 */
static void Compass_CalFinish(void)
{
    /*span 必须用 int: 极限情况 32767-(-32768)=65535
     *装进 int16_t 会回绕成 -1, 导致满圈旋转被误判"摆幅不足"
     */
    int spanx;
    int spany;

    cp_cal_state = 0;
    spanx = cp_maxx - cp_minx;
    spany = cp_maxy - cp_miny;

    /*摆幅太小 = 没转够/转太快, 结果不可信, 保留旧偏移*/
    if (spanx < CP_CAL_MIN_SPAN && spany < CP_CAL_MIN_SPAN)
    {
        cp_cal_result = 2;              /*提示 "too fast"*/
    }
    else
    {
        /*硬磁偏移 = 极值中点 (手册/通用做法)
         *int16 除法向零截断, ±1 LSB 的误差可忽略
         */
        cp_cal_offx = (int16_t)((cp_maxx + cp_minx) / 2);
        cp_cal_offy = (int16_t)((cp_maxy + cp_miny) / 2);
        cp_cal_result = 1;              /*成功 (暂无专属提示, 数字会立刻变准)*/
    }
    cp_redraw = 1;
}

/**
 * 指南针页面主节拍 — 由 Watch_Tick 在 Compass 页面调用
 */
void Compass_Tick(void)
{
    uint8_t  ret;                       /*QMC5883P_ReadRaw 返回值*/
    int16_t  fwd, right;                /*减偏移后的前/右水平分量*/
    float    hdeg;                      /*浮点航向角 (含磁偏角修正后)*/

    /*══ 首次进入: 初始化传感器 (懒初始化) ══
     *必须在 OLED_Init 之后 (I2C 引脚是它配的), 主循环里此时必然已初始化
     *Init 内部会查 CHIP_ID, 模块没接也会安全返回 0
     */
    if (cp_first_enter)
    {
        cp_first_enter = 0;
        cp_sensor_ok   = QMC5883P_Init();
        if (!cp_sensor_ok)
        {
            /*初始化失败: 先扫一遍总线再画诊断页,
             *首帧就带扫描结果, 不用等 0.5s 重扫周期
             */
            cp_scan_n = QMC5883P_ScanBus(cp_found, 4);
            Compass_DisplayNoSensor();
            return;
        }
        cp_redraw = 1;
    }

    /*══ 防御: 非指南针页面不执行 (与其他叶子模块同款) ══*/
    if (NowState != Compass)
        return;

    /*══ 模块不在线: 诊断态 — 重扫总线/重试初始化/退出 ══*/
    if (!cp_sensor_ok)
    {
        if (Key_IsPressed(KEY_BACK))
        {
            cp_first_enter = 1;         /*下次进入重新探测*/
            Watch_ExitLeafPage();
            return;
        }

        /*0.5s 自动重扫一次; 按 OK 可以立即重扫
         *(插拔线/重焊接后不用退页重进, 盯着 SCAN 行变化即可)
         */
        cp_scan_cnt++;
        if (cp_scan_cnt < 50 && !Key_IsPressed(KEY_OK))
            return;                     /*未到重扫时间, 本tick结束*/

        cp_scan_cnt = 0;
        cp_scan_n = QMC5883P_ScanBus(cp_found, 4);

        /*只有看到 0x2C (P版芯片) 才重试初始化;
         *0x0D 是 L 版芯片, 协议完全不同, 重试也不会成功
         */
        if (Compass_AddrFound(0x2C))
            cp_sensor_ok = QMC5883P_Init();

        if (!cp_sensor_ok)
        {
            Compass_DisplayNoSensor();  /*刷新诊断画面*/
            return;
        }
        cp_redraw = 1;                  /*初始化成功, 本tick继续走正常流程*/
    }

    /*══ 读传感器 (10Hz 出新数据, 其余 tick 直接返回0) ══*/
    ret = QMC5883P_ReadRaw(&cp_rawx, &cp_rawy, &cp_rawz);
    if (ret == 1)
    {
        /*── 有新数据 ──*/
        if (cp_cal_state == 1)
        {
            /*校准中: 用原始值刷新 X/Y 极值 (OVFL 坏值驱动里已丢弃)*/
            if (cp_rawx < cp_minx)  cp_minx = cp_rawx;
            if (cp_rawx > cp_maxx)  cp_maxx = cp_rawx;
            if (cp_rawy < cp_miny)  cp_miny = cp_rawy;
            if (cp_rawy > cp_maxy)  cp_maxy = cp_rawy;
        }

        /*── 航向解算 ──
         *1) 减硬磁偏移 → 数据圆心回到原点
         *2) 经安装方向宏映射成"前/右"两个水平分量
         *3) atan2(右, 前) → 0°=北, 顺时针 0~360
         */
        fwd   = CP_MOUNT_FWD(cp_rawx - cp_cal_offx, cp_rawy - cp_cal_offy);
        right = CP_MOUNT_RIGHT(cp_rawx - cp_cal_offx, cp_rawy - cp_cal_offy);

        hdeg = atan2f((float)right, (float)fwd) * 180.0f / 3.14159265f
             + CP_DECLINATION_DEG;
        if (hdeg < 0.0f)
            hdeg += 360.0f;
        else if (hdeg >= 360.0f)
            hdeg -= 360.0f;

        cp_heading_deg = (uint16_t)(hdeg + 0.5f);       /*四舍五入到整度*/
        if (cp_heading_deg >= 360)
            cp_heading_deg = 0;
        cp_cardinal = Compass_CardinalIndex(cp_heading_deg);

        cp_redraw = 1;                  /*新数据 → 重绘盘面*/
    }
    /*ret==0: 无新数据, 不重绘 (省电防闪)
     *ret==2: 溢出丢弃, 保持上一帧画面
     */

    /*══ 按键处理 ══*/
    if (Key_IsPressed(KEY_BACK))
    {
        if (cp_cal_state == 1)
        {
            cp_cal_state = 0;           /*校准中按BACK: 放弃校准*/
            cp_redraw = 1;
        }
        else
        {
            /*正常状态按BACK: 传感器挂起省电 + 退出页面*/
            cp_cal_state = 0;
            QMC5883P_Suspend();
            cp_first_enter = 1;         /*下次进入重新初始化/探测*/
            Watch_ExitLeafPage();
            return;                     /*NowState已变, 不再绘制*/
        }
    }
    else if (Key_IsLongPress(KEY_OK))
    {
        /*长按OK: 正常态→开始校准 (校准态长按无操作, 短按OK才是结束)*/
        if (cp_cal_state == 0)
            Compass_CalStart();
    }
    else if (Key_IsPressed(KEY_OK))
    {
        /*短按OK: 仅校准态有效 → 结束校准并应用偏移*/
        if (cp_cal_state == 1)
            Compass_CalFinish();
    }

    /*══ 校准提示闪烁定时 (0.5秒翻转, 同其他模块套路) ══*/
    if (cp_cal_state == 1)
    {
        if (cp_blink_cnt < 50)
            cp_blink_cnt++;
        else
        {
            cp_blink = !cp_blink;
            cp_blink_cnt = 0;
            cp_redraw = 1;
        }
    }
    else
    {
        cp_blink = 1;                   /*非校准态强制亮, 防止停在灭半周*/
        cp_blink_cnt = 0;
    }

    /*══ 重绘 ══*/
    if (cp_redraw)
        Compass_Display();
}
