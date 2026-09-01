/**
 * ============================================================
 *  TimeSetting.c — 时间设置模块 (自治状态机 + 自绘显示)
 *
 *  设计要点:
 *    1. 完全自治: 内部管理 redraw/blink 标志, 不依赖 watch.c 的私有变量
 *    2. 双层状态: NoEditing(选字段) ↔ Editing(改字段)
 *    3. 跨文件通信: 通过 watch.h 的 extern NowState 读菜单状态,
 *       通过 Watch_ExitLeafPage() 退回上级菜单 (不直接碰 watch.c 私有变量)
 *    4. 离开自动复位: Tick 开头检测 NowState!=Time 时重置内部状态
 *
 *  按键映射:
 *    UP/DOWN : NoEditing→选字段(Year↔Second循环) / Editing→+1/-1
 *    OK      : NoEditing→Editing / Editing→确认跳下一字段回NoEditing
 *    BACK    : Editing→NoEditing / NoEditing→退出Time页(写RTC)
 *
 *  显示布局 (128x64):
 *    Y=0   标题 "TimeSetting" (6x8)
 *    Y=9   日期 "YYYY-MM-DD"  (8x16, NoEditing态反相框选中)
 *    Y=26  时间 "HH:MM:SS"    (8x16, Editing态闪烁)
 * ============================================================
 */

#include "TimeSetting.h"
#include "watch.h"          /* NowState (extern), Watch_ExitLeafPage, Time 枚举值 */
#include "Key.h"            /* Key_IsPressed, KEY_xxx */
#include "OLED.h"           /* OLED_xxx */
#include "MyRTC.h"          /* MyRTC_Time[], MyRTC_ReadTime, MyRTC_SetTime */

/*══════════════════════════════════════════════════════════════
 *  字段索引枚举 (仅本文件可见)
 *══════════════════════════════════════════════════════════════*/
enum TimeSettingSearchindex{
	Year=1,
	Month,
	Day,
	Hour,
	Minute,
	Second,
	Editing,
	NoEditing,
};

/*══════════════════════════════════════════════════════════════
 *  私有状态变量 (仅本文件可见, static)
 *  全部用 static 修饰, 外部文件无法直接访问 ——"能藏就藏"
 *══════════════════════════════════════════════════════════════*/
static char    TimeSettingSelected  = Year;       /*当前选中字段 (Year~Second)*/
static char    TimeSettingState     = NoEditing;  /*编辑状态 (Editing/NoEditing)*/
static char    IsFirstEnterTimeSetting = 1;        /*首次进入标志 (1=需读RTC)*/
static char    TimeChangeFlag       = 0;           /*加减请求 (1=+1, 2=-1)*/
static uint8_t IsTimeStateChanged   = 0;           /*状态变化标志 (触发重绘)*/
static uint8_t blink_toggle         = 1;           /*闪烁标志: 1=显示, 0=隐藏*/

/*本模块自己的重绘标志 (不依赖 watch.c)*/
static uint8_t ts_need_redraw = 1;
/*闪烁节拍计数器 (50×10ms=0.5秒翻转)*/
static uint8_t ts_blink_cnt = 0;

/*重绘标志复用 watch.c 的 (通过 extern 无法直接访问)
 *实际上 TimeSetting 需要 watch.c 配合: Watch_Tick 的变化检测会置 watch_need_redraw
 *但 TimeSetting 内部的 flag 消费/闪烁也需要触发重绘, 用 IsTimeStateChanged 兜底
 */


/*══════════════════════════════════════════════════════════════
 *  按键处理 (私有, 仅本文件内 TimeSetting_Tick 调用)
 *══════════════════════════════════════════════════════════════*/
static void TimeSetting_HandleKey(void)
{
	if(Key_IsPressed(KEY_UP))
	{
		if(TimeSettingState!=Editing)
		{
			TimeSettingSelected++;
			if(TimeSettingSelected>6)
				TimeSettingSelected=1;
		}
		else
		{
			TimeChangeFlag=1;
		}
		IsTimeStateChanged=1;
	}
	else if(Key_IsPressed(KEY_DOWN))
	{
		if(TimeSettingState!=Editing)
		{
			TimeSettingSelected--;
			if(TimeSettingSelected<1)
				TimeSettingSelected=6;
		}
		else
		{
			TimeChangeFlag=2;
		}
		IsTimeStateChanged=1;
	}
	else if(Key_IsPressed(KEY_OK))
	{
		if(IsFirstEnterTimeSetting==0)
		{
			if(TimeSettingState == NoEditing)
			{
				TimeSettingState = Editing;
			}
			else
			{
				TimeSettingState = NoEditing;
				TimeSettingSelected++;           /*确认跳下一字段*/
				if(TimeSettingSelected > Second)
					TimeSettingSelected = Year;
			}
		}
		IsTimeStateChanged=1;
	}
	else if(Key_IsPressed(KEY_BACK))
	{
		if(TimeSettingState==Editing)
		{
			TimeSettingState=NoEditing;
		}
		else
		{
			/*退出Time页: 保存改动 + 委托watch.c退回上级菜单
			 *内部状态重置,离开检查
			 */
			MyRTC_SetTime();
			Watch_ExitLeafPage();
			IsFirstEnterTimeSetting=1;
			TimeSettingSelected     = Year;
			TimeSettingState        = NoEditing;
			IsFirstEnterTimeSetting = 1;
			TimeChangeFlag          = 0;
			IsTimeStateChanged      = 0;
			blink_toggle            = 1;
			ts_blink_cnt            = 0;
			ts_need_redraw          = 1;  
			return;   /*NowState已变, 后续不执行*/
		}
		IsTimeStateChanged=1;
	}
}


/*══════════════════════════════════════════════════════════════
 *  显示绘制 (私有, 仅本文件内 TimeSetting_Tick 调用)
 *══════════════════════════════════════════════════════════════*/
static void TimeSetting_Display(void)
{
	OLED_Clear();
	OLED_ShowString(30,0,"TimeSetting",OLED_6X8);
	OLED_ShowString(18, 9, "XXXX-XX-XX",OLED_8X16);
	OLED_ShowString(26, 26, "XX:XX:XX",OLED_8X16);
	OLED_ShowNum(18, 9, MyRTC_Time[0], 4,OLED_8X16);		/*年*/
	OLED_ShowNum(58, 9, MyRTC_Time[1], 2,OLED_8X16);		/*月*/
	OLED_ShowNum(82, 9, MyRTC_Time[2], 2,OLED_8X16);		/*日*/
	OLED_ShowNum(26, 26, MyRTC_Time[3], 2,OLED_8X16);		/*时*/
	OLED_ShowNum(50, 26, MyRTC_Time[4], 2,OLED_8X16);		/*分*/
	OLED_ShowNum(74, 26, MyRTC_Time[5], 2,OLED_8X16);		/*秒*/

	if(TimeSettingState==NoEditing)
	{
		/*NoEditing态: 当前选中字段画反相框 (选中提示)*/
		switch(TimeSettingSelected)
		{
			case Year:  OLED_ReverseArea(18, 9, 32, 16);break;
			case Month: OLED_ReverseArea(58, 9, 16, 16);break;
			case Day:   OLED_ReverseArea(82, 9, 16, 16);break;
			case Hour:  OLED_ReverseArea(26, 26, 16, 16);break;
			case Minute:OLED_ReverseArea(50, 26, 16, 16);break;
			case Second:OLED_ReverseArea(74, 26, 16, 16);break;
		}
	}
	if (TimeSettingState == Editing)
	{
		/*Editing态: blink_toggle==1时画反相框(闪烁亮半周), ==0时不画(暗半周)*/
		if(blink_toggle)
		{
			switch(TimeSettingSelected)
			{
				case Year:  OLED_ReverseArea(18, 9, 32, 16);break;
				case Month: OLED_ReverseArea(58, 9, 16, 16);break;
				case Day:   OLED_ReverseArea(82, 9, 16, 16);break;
				case Hour:  OLED_ReverseArea(26, 26, 16, 16);break;
				case Minute:OLED_ReverseArea(50, 26, 16, 16);break;
				case Second:OLED_ReverseArea(74, 26, 16, 16);break;
			}
		}
	}
	OLED_Update();
}


/*══════════════════════════════════════════════════════════════
 *  时间设置主节拍 — 由 Watch_Tick 在 Time 页面调用
 *══════════════════════════════════════════════════════════════*/

/*各字段的最小/最大值 (索引0~5 对应 年~秒)*/
static const uint16_t tmin[6] = {2000, 1,  1,  0,  0,  0};
static const uint16_t tmax[6] = {2099, 12, 31, 23, 59, 59};


void TimeSetting_Tick(void)
{
	uint8_t idx;   /*当前字段对应的 MyRTC_Time 数组下标 (0~5)*/

	/*══ 离开Time页时复位内部状态 (下次进入干净) ══
	 *与Stopwatch同模式: 检测NowState!=Time就重置, return
	 *这样无论从哪条路径离开都能保证状态干净
	 */
	if (NowState != Time)
	{
		return;
	}

	/*══ 首次进入: 从RTC读取当前时间 ══*/
	if (IsFirstEnterTimeSetting)
	{
		MyRTC_ReadTime();
		TimeSettingSelected = Year;
		ts_need_redraw = 1;
	}

	/*══ 第1步: 按键处理 ══*/
	TimeSetting_HandleKey();
	/*HandleKey里BACK可能已退出Time页, 退出后不再画*/
	if (NowState != Time) return;

	/*══ 第2步: 闪烁定时 (仅Editing态, 0.5秒翻转一次) ══*/
	if(TimeSettingState == Editing)
	{
		if(ts_blink_cnt < 50)
			ts_blink_cnt++;
		else
		{
			blink_toggle = !blink_toggle;
			ts_blink_cnt = 0;
			ts_need_redraw = 1;
		}
	}
	else
	{
		/*NoEditing态强制显示, 避免从Editing退出时停在隐藏半周*/
		blink_toggle = 1;
	}

	/*══ 第3步: 消费 TimeChangeFlag (Editing态下UP/DOWN的加减请求) ══*/
	if (TimeChangeFlag)
	{
		idx = TimeSettingSelected - 1;  /*Year=1→idx0, ..., Second=6→idx5*/

		if (TimeChangeFlag == 1)        /*UP: +1*/
		{
			if (MyRTC_Time[idx] >= tmax[idx])
				MyRTC_Time[idx] = tmin[idx];
			else
				MyRTC_Time[idx]++;
		}
		else                            /*DOWN: -1*/
		{
			if (MyRTC_Time[idx] <= tmin[idx])
				MyRTC_Time[idx] = tmax[idx];
			else
				MyRTC_Time[idx]--;
		}
		TimeChangeFlag = 0;             /*消费完毕, 清标志*/
		ts_need_redraw = 1;
	}

	/*══ 第4步: 状态变化触发重绘 ══*/
	if(IsTimeStateChanged)
	{
		ts_need_redraw = 1;
		IsTimeStateChanged = 0;
	}

	/*══ 第5步: 重绘 (用本模块的 ts_need_redraw 标志) ══*/
	if (ts_need_redraw)
	{
		TimeSetting_Display();
		ts_need_redraw = 0;
	}

	IsFirstEnterTimeSetting = 0;
}
