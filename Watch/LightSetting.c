#include "LightSetting.h"
#include "watch.h"          /* NowState (extern), Watch_ExitLeafPage, Light 枚举值 */
#include "Key.h"            /* Key_IsPressed, KEY_xxx */
#include "OLED.h"           /* OLED_xxx */

/*══════════════════════════════════════════════════════════════
 *  亮度档位表
 *  共11档 (0~10), 每档对应一个 0~255 的对比度值
 *  分布偏非线性: 低档间隔小方便微调, 高档间隔大避免过曝
 *  档位 0 = 全黑, 档位 10 = 最亮 (与OLED_Init的0xCF一致)
 *══════════════════════════════════════════════════════════════*/
#define LIGHT_LEVELS  11
static const uint8_t light_table[LIGHT_LEVELS] = {
    0x00,   /*档0:  全黑*/
    0x10,   /*档1*/
    0x30,   /*档2*/
    0x50,   /*档3*/
    0x70,   /*档4*/
    0x90,   /*档5*/
    0xB0,   /*档6*/
    0xCF,   /*档7:  默认亮度 (与OLED_Init一致)*/
    0xE0,   /*档8*/
    0xF0,   /*档9*/
    0xFF,   /*档10: 最亮*/
};

/*══════════════════════════════════════════════════════════════
 *  私有状态变量
 *══════════════════════════════════════════════════════════════*/
static uint8_t  ls_level       = 7;    /*当前档位 (0~10), 初值7=默认亮度*/
static uint8_t  ls_redraw      = 1;    /*重绘标志*/
static uint8_t  ls_first_enter = 1;    /*首次进入标志*/

/*条形图布局参数*/
#define BAR_X       14              /*条形图左上角X (条总宽100, 居中(128-100)/2=14)*/
#define BAR_Y       28              /*条形图Y*/
#define BAR_W       100             /*条形图总宽 (10格 × 10px)*/
#define BAR_H       12              /*条形图高*/
#define BAR_CELL    10              /*每格宽度*/

/**
 * 亮度调节页面显示
 * 标题 + 当前档位百分比 + 条形图
 */
static void LightSetting_Display(void)
{
    uint8_t i;
    uint8_t filled;         /*条形图已填充格数*/
    uint8_t percent;        /*百分比 (0~100)*/

    ls_redraw = 0;
    OLED_Clear();

    /*── 标题 "Brightness" 居中 (10字符 × 8px = 80, (128-80)/2=24) ──*/
    OLED_ShowString(24, 0, "Brightness", OLED_8X16);
    OLED_ReverseArea(0, 0, 128, 16);

    /*── 百分比数字 居中显示 ──*/
    percent = (uint8_t)(ls_level * 100 / (LIGHT_LEVELS - 1));
    OLED_ShowString(48, 17, "Lv", OLED_6X8);                 /*"Lv" 标签*/
    OLED_ShowNum(64, 17, ls_level, 2, OLED_6X8);             /*档位数字 00~10*/
    OLED_ShowString(80, 17, "(",  OLED_6X8);
    OLED_ShowNum(86, 17, percent, 3, OLED_6X8);              /*百分比 0~100*/
    OLED_ShowString(104, 17, "%)", OLED_6X8);

    /*── 条形图: 外框 + 填充格 ──*/
    OLED_DrawRectangle(BAR_X, BAR_Y, BAR_W, BAR_H, OLED_UNFILLED);
    filled = ls_level;       /*档位0~10 对应填充0~10格*/
    for (i = 0; i < LIGHT_LEVELS - 1; i++)   /*画10格填充*/
    {
        if (i < filled)
        {
            /*填充: 用反相区域画一格黑块*/
            OLED_ReverseArea(BAR_X + 1 + i * BAR_CELL,BAR_Y + 1,BAR_CELL - 1,BAR_H - 2);
        }
    }

    /*── 底部操作提示 ──*/
    OLED_ShowString(16, 48, "UP/DN  OK/BACK", OLED_6X8);

    OLED_Update();
}

/**
 * 亮度调节主节拍
 * 由 Watch_Tick 在 NowState==Light 时调用
 *
 * 按键逻辑:
 *   UP:   档位 +1 (到顶循环回0)
 *   DOWN: 档位 -1 (到底循环回10)
 *   OK / BACK: 退出页面 (亮度立即生效, 退出后保持)
 */
void LightSetting_Tick(void)
{
    /*防御: 非亮度调节页面不执行*/
    if (NowState != Light) return;

    /*首次进入: 应用当前档位并强制绘制一帧*/
    if (ls_first_enter)
    {
        OLED_SetBrightness(light_table[ls_level]);
        LightSetting_Display();
        ls_first_enter = 0;
    }

    /*══ 按键处理 ══*/
    if (Key_IsPressed(KEY_UP))
    {
        /*UP: +1, 到顶循环回0*/
        if (ls_level >= LIGHT_LEVELS - 1)
            ls_level = 0;
        else
            ls_level++;
        OLED_SetBrightness(light_table[ls_level]);   /*立即生效*/
        ls_redraw = 1;
    }
    else if (Key_IsPressed(KEY_DOWN))
    {
        /*DOWN: -1, 到底循环回10*/
        if (ls_level == 0)
            ls_level = LIGHT_LEVELS - 1;
        else
            ls_level--;
        OLED_SetBrightness(light_table[ls_level]);   /*立即生效*/
        ls_redraw = 1;
    }
    else if (Key_IsPressed(KEY_OK) || Key_IsPressed(KEY_BACK))
    {
        /*OK或BACK: 退出, 亮度保持当前值*/
        ls_first_enter = 1;          /*重置: 下次进入会重新初始化*/
        Watch_ExitLeafPage();
        return;                      /*NowState已变, 不再绘制*/
    }

    /*══ 重绘 ══*/
    if (ls_redraw)
    {
        LightSetting_Display();
    }
}
