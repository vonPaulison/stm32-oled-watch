/**
 * ============================================================
 *  Stopwatch.c — 秒表模块 (自治状态机 + 自绘显示)
 *
 *  设计要点:
 *    1. 完全自治: 内部管理 redraw 标志, 不依赖 watch.c 的私有变量
 *    2. 计时基准: 复用 Watch_Tick 的 10ms 节拍, 每 tick 累加 sw_ms
 *    3. 状态机: IDLE → RUNNING → PAUSED → RUNNING (循环) / → IDLE (复位)
 *    4. 跨文件通信: 通过 watch.h 的 extern NowState 读菜单状态,
 *       通过 Watch_ExitLeafPage() 退回上级菜单 (不直接碰 watch.c 私有变量)
 *
 *  按键映射:
 *    OK    : IDLE→RUNNING / RUNNING→PAUSED / PAUSED→RUNNING (启停切换)
 *    DOWN  : 任意非RUNNING态 → IDLE (复位清零)
 *    BACK  : 退出秒表页, 回到 Menu
 *
 *  显示布局 (128x64):
 *    Y=0   标题 "Stopwatch" (8x16, 居中)
 *    Y=24  时间 "MM:SS.ms"  (8x16, 居中, PAUSED态闪烁)
 *    Y=50  状态 "RUN/PAUSE/IDLE" (6x8, 居中)
 *
 *  时间精度: 10ms (显示 ms 两位, 实际是 10ms 单位)
 * ============================================================
 */

#include "Stopwatch.h"
#include "watch.h"          /* NowState (extern), Watch_ExitLeafPage, Stopwatch 枚举值 */
#include "Key.h"            /* Key_IsPressed, KEY_xxx */
#include "OLED.h"           /* OLED_xxx */
#include "Timer.h"          /* Timer_GetTick() — 硬件 1ms 精度计时*/
#include "MyRTC.h"

/*══════════════════════════════════════════════════════════════
 *  私有状态定义 (仅本文件可见)
 *══════════════════════════════════════════════════════════════*/
enum {
    SW_IDLE = 0,            /*初始/已复位: 显示 00:00.00*/
    SW_RUNNING,             /*计时中: 持续累加 + 刷新*/
    SW_PAUSED,              /*已暂停: 数字闪烁, 可复位或继续*/
};

/*显示坐标参数 (8x16字体每字符8px宽)*/
#define SW_TITLE_Y     0                /*标题行*/
#define SW_TIME_Y      24               /*时间数字行*/
#define SW_STATE_Y     50               /*状态提示行*/
#define SW_TIME_X      32               /*"MM:SS.ms" 8字符×8px=64px, 居中(128-64)/2=32*/
#define SW_TIME_WIDTH  64               /*时间数字总宽 (用于局部刷新)*/


/*══════════════════════════════════════════════════════════════
 *  私有状态变量 (仅本文件可见, static)
 *  注意: 全部用 static 修饰, 外部文件无法直接访问
 *        这是模块封装的核心 ——"能藏就藏"
 *══════════════════════════════════════════════════════════════*/
static uint32_t sw_accumulated_ms = 0;   /*暂停前累计的毫秒数 (支持暂停后继续)*/
static uint32_t sw_resume_tick    = 0;   /*本次开始/继续时的 Timer_GetTick() 值*/
static uint32_t sw_displayed_ms   = 0;   /*上次显示的毫秒数 (用于判断要不要重绘)*/
static uint8_t  sw_state          = SW_IDLE;
static uint8_t  sw_redraw         = 1;   /*全屏重绘标志 (首次进入/状态切换时置1)*/
static uint8_t  sw_isfirstenter   = 1;
static uint32_t cur_ms;

/**
 * 画时间数字到显存 (不清屏, 不Update, 供局部刷新调用)
 * 把"画时间"单独抽出来, 全屏和局部刷新都能复用
 */
static void StopWatch_DrawTime(uint32_t cur_ms)
{
    uint16_t mm, ss, ms10;
	
    mm   = (cur_ms / 60000) % 60;
    ss   = (cur_ms / 1000)  % 60;
    ms10 = (cur_ms / 10)    % 100;

    OLED_ShowNum (SW_TIME_X,      SW_TIME_Y, mm,   2, OLED_8X16);
    OLED_ShowChar(SW_TIME_X + 16, SW_TIME_Y, ':',  OLED_8X16);
    OLED_ShowNum (SW_TIME_X + 24, SW_TIME_Y, ss,   2, OLED_8X16);
    OLED_ShowChar(SW_TIME_X + 40, SW_TIME_Y, ':',  OLED_8X16);
    OLED_ShowNum (SW_TIME_X + 48, SW_TIME_Y, ms10, 2, OLED_8X16);
}


/**
 * 全屏重绘: 标题 + 时间 + 状态 (首次进入/状态切换时用)
 * - 调 OLED_Clear 清整屏显存
 * - 画全部内容
 * - OLED_Update 发整屏
 */
static void StopWatch_Display(void)
{

    if (!sw_redraw) return;
    sw_redraw = 0;
    OLED_Clear();
    /*── 标题 "Stopwatch" 居中 (9字符×8px=72px, X=(128-72)/2=28) ──*/
    OLED_ShowString(28, SW_TITLE_Y, "Stopwatch", OLED_8X16);

    /*重新画时间到显存*/
    StopWatch_DrawTime(cur_ms);

    /*── 状态提示 ──*/
    if (sw_state == SW_RUNNING)
        OLED_ShowString((128 - 3*6)/2, SW_STATE_Y, "RUN",   OLED_6X8);
    else if (sw_state == SW_PAUSED)
        OLED_ShowString((128 - 5*6)/2, SW_STATE_Y, "PAUSE", OLED_6X8);
    else
        OLED_ShowString((128 - 4*6)/2, SW_STATE_Y, "IDLE",  OLED_6X8);
    OLED_Update();
}

/**
 * 秒表主节拍 — 由 Watch_Tick 在 Stopwatch 页面调用
 */
void Stopwatch_Tick(void)
{
    /*══ 离开秒表页时复位内部状态 (下次进入干净) ══*/
    if (NowState != Stopwatch)
    {
        return;
    }
    /*══ 计时: RUNNING 态实时算经过时间 ══
     * 不再 sw_ms++ (那个会被 OLED 拖慢)
     * 改成: 累计时间 = 暂停前累计 + (现在 - 本次开始tick)
     * 硬件 TIM2 中断保证 Timer_GetTick() 精度 1ms
     *
     * 刷新策略 (性能优化):
     *   - 时间变化 (每10ms) → 只刷时间区域 (sw_time_dirty, ~2ms)
     *   - 状态切换/首次进入 → 全屏重绘 (sw_redraw, ~15ms)
     *   这样RUNNING态每个tick只花2ms, 不再拖慢主循环
     */
    if (sw_state == SW_RUNNING)
    {
        cur_ms = sw_accumulated_ms + (Timer_GetTick() - sw_resume_tick);
        /*只在显示值变化时标记时间区域脏 (减少无谓刷新)
         *显示精度 10ms, 所以每 10ms 变化才刷
         */
        if (cur_ms != sw_displayed_ms)
        {
            sw_displayed_ms = cur_ms;
            sw_redraw = 1;          /*时间需要刷新*/
        }
    }
	else
	{
		cur_ms = sw_accumulated_ms;
	}

    /*══ 按键处理 ══*/
    if (Key_IsPressed(KEY_OK))
    {
        if (sw_isfirstenter == 0)
        {
            if (sw_state == SW_IDLE)
            {
                /*开始计时: 记录起始tick, 清累计*/
                sw_accumulated_ms = 0;
                sw_resume_tick    = Timer_GetTick();
                sw_state          = SW_RUNNING;
            }
            else if (sw_state == SW_RUNNING)
            {
                /*暂停: 把经过的时间累加到 accumulated, 固定下来*/
                sw_accumulated_ms += (Timer_GetTick() - sw_resume_tick);
                sw_state = SW_PAUSED;
            }
            else /*SW_PAUSED*/
            {
                /*继续: 记录新的起始tick, accumulated 保留*/
                sw_resume_tick = Timer_GetTick();
                sw_state       = SW_RUNNING;
            }
            sw_redraw = 1;
        }
    }
    else if (Key_IsPressed(KEY_DOWN))
    {
        /*复位: 仅非RUNNING态允许*/
        if (sw_state != SW_RUNNING)
        {
            cur_ms = 0;
			sw_accumulated_ms = 0;
            sw_state          = SW_IDLE;
            sw_redraw = 1;
        }
    }
    else if (Key_IsPressed(KEY_BACK))
    {
		/*退出后全部重置*/
        sw_accumulated_ms = 0;
        sw_resume_tick    = 0;
        sw_state          = SW_IDLE;
        sw_isfirstenter   = 1;
        sw_redraw         = 1;
		sw_accumulated_ms = 0;
        Watch_ExitLeafPage();
        return;
    }

    if (sw_isfirstenter == 1)
    {
        sw_redraw = 1;              /*首次进入强制全屏重绘*/
        sw_isfirstenter = 0;
    }

    /*══ 重绘: 优先全屏, 否则局部 ══
     * sw_redraw (全屏) 优先级高于 sw_time_dirty (局部)
     * 因为状态切换时整个画面都要重画, 只刷时间区域会留下旧状态文字
     */
    if (sw_redraw == 1)
    {
        StopWatch_Display();        /*全屏 (~15ms, 但只在状态切换时调)*/
        sw_redraw = 0;
       
    }
    
}
