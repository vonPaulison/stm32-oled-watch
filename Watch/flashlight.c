#include "flashlight.h"
#include "watch.h"          /* NowState (extern), Watch_ExitLeafPage, Normal/Glitter 枚举值 */
#include "Key.h"            /* Key_IsPressed, KEY_xxx */
#include "OLED.h"           /* OLED_xxx */
#include "LED.h"            /* Flashlight_xxx */

/*══════════════════════════════════════════════════════════════
 *  私有状态变量 (仅本文件可见, static)
 *══════════════════════════════════════════════════════════════*/
static uint8_t fl_switch  = 0;   /*开关状态: 1=开, 0=关*/
static uint8_t fl_redraw  = 1;   /*本模块重绘标志, 1=需要重画*/
static uint8_t fl_glitter = 0;   /*闪烁模式节拍: 1=灯亮, 0=灯灭*/
static uint8_t fl_tick_cnt = 0;  /*闪烁计时 (10ms 一次, 累计到阈值翻转)*/
static uint8_t fl_first_enter  = 1;   /*是否第一次进入, 1=yes*/
/*闪烁模式周期: 10 × 10ms = 0.1 秒, 人眼可见且刺眼*/
#define FL_GLITTER_PERIOD 10

/**
 * 手电筒页面显示
 * 只在 fl_redraw==1 时执行, 避免每 tick 都刷屏
 */
static void Flashlight_Display(void)
{
    fl_redraw = 0;
    OLED_Clear();
    if (NowState == Normal)
    {
        OLED_ShowString(44, 0, "Light", OLED_8X16);
		OLED_ReverseArea(0, 0, 128, 18);
        if (fl_switch)
        {
            OLED_ShowString(56, 28, "ON", OLED_8X16);
            OLED_ReverseArea(53, 28, 22, 18);  /*常亮模式: 反相高亮 ON*/
        }
        else
        {
            OLED_ShowString(52, 28, "OFF", OLED_8X16);
        }
    }
    else /*Glitter*/
    {
        OLED_ShowString(36, 0, "Glitter", OLED_8X16);
		OLED_ReverseArea(0, 0, 128, 18);
        if (fl_switch)
        {
            /*闪烁模式: 显示 RUNNING 提示*/
            OLED_ShowString(56, 28, "ON", OLED_8X16);
			OLED_ReverseArea(53, 28, 22, 18);  /*爆闪模式: 反相高亮 ON*/
        }
        else
        {
            OLED_ShowString(52, 28, "OFF", OLED_8X16);
        }
    }
    OLED_Update();
}

/**
 * 手电筒主节拍
 * 由 Watch_Tick 在 NowState==Normal 或 NowState==Glitter 时调用
 *
 * 按键逻辑:
 *   Normal 模式:  OK 开灯, BACK 关灯退出
 *   Glitter 模式: OK 开始/停止闪烁, BACK 停止并退出
 */
void Flashlight_Tick(void)
{
	if(fl_first_enter)
	{
		Flashlight_Display();
		fl_first_enter=0;
	}
    /*防御: 非手电筒页面不执行*/
    if (NowState != Normal && NowState != Glitter) return;

    /*══ 闪烁模式: 时间到就翻转灯亮/灭 ══*/
    if (NowState == Glitter && fl_switch)
    {
        if (++fl_tick_cnt >= FL_GLITTER_PERIOD)
        {
            fl_tick_cnt = 0;
            fl_glitter = !fl_glitter;
            if (fl_glitter)
                Flashlight_ON();
            else
                Flashlight_OFF();
            fl_redraw = 1;
        }
    }

    /*══ 按键处理 ══*/
    if (Key_IsPressed(KEY_OK))
    {
        fl_switch = !fl_switch;   /*OK 切换开关状态*/

        if (NowState == Normal)
        {
            if (fl_switch)
                Flashlight_ON();
            else
                Flashlight_OFF();
        }
        else /*Glitter: 开始或停止闪烁*/
        {
            if (fl_switch)
            {
                fl_tick_cnt = 0;
                fl_glitter  = 1;
                Flashlight_ON();
            }
            else
            {
                Flashlight_OFF();
            }
        }
        fl_redraw = 1;
    }
    else if (Key_IsPressed(KEY_BACK))
    {
        /*BACK: 强制关灯并退出页面*/
        fl_switch = 0;
        fl_glitter = 0;
        fl_tick_cnt = 0;
        Flashlight_OFF();
		fl_first_enter=1;
        Watch_ExitLeafPage();
        return;             /*NowState已变, 不再绘制*/
    }

    /*══ 重绘 ══*/
    if(fl_redraw)
		Flashlight_Display();
}
