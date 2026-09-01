#include "Countdown.h"
#include "watch.h"          /* NowState (extern), Watch_ExitLeafPage, Time 枚举值 */
#include "Key.h"            /* Key_IsPressed, KEY_xxx */
#include "OLED.h"           /* OLED_xxx */
#include "MyRTC.h"          /* MyRTC_Time[], MyRTC_ReadTime, MyRTC_SetTime */
#include "Timer.h"
/*══════════════════════════════════════════════════════════════
 *  字段索引枚举 (仅本文件可见)
 *══════════════════════════════════════════════════════════════*/
enum Cd_Searchindex{
	Hour=1,
	Minute,
	Second,
	Cd_Start,
	Editing,
	Cd_Idle,            /*初始/已复位: 显示设定值*/
    Cd_Running,         /*计时中: 持续刷新剩余时间*/
    Cd_Pause, 			/*暂停中: 时间冻结, 整体闪烁*/
    Cd_Finished,        /*到时: 显示00:00:00 + Time Up! 闪烁*/
};

/*══════════════════════════════════════════════════════════════
 *  私有状态变量 (仅本文件可见, static)
 *  全部用 static 修饰, 外部文件无法直接访问 ——"能藏就藏"
 *══════════════════════════════════════════════════════════════*/
static char    Cd_Selected  	= Second;              /*当前选中字段 (Hour~Second)*/
static char    Cd_State     	= Cd_Idle;    		   /*计时状态 (Cd_Running/Cd_Pause)*/
static char    IsFirstEnter 	= 1;                   /*首次进入标志 */
static char    TimeChangeFlag   = 0;           		   /*加减请求 (1=+1, 2=-1)*/

static int8_t    showtime[3]={0,0,0};

static uint32_t set_ms=0;
static uint32_t begin_ms=0;
static uint32_t past_ms=0;
static uint32_t remaintime_ms=0;

/*本模块的重绘标志 */
static uint8_t cd_redraw = 1;

/*闪烁相关 (Editing/Pause/Finished 态用, 0.5秒翻转一次)*/
static uint8_t cd_blink     = 1;
static uint8_t cd_blink_cnt = 0;

/*时间显示坐标 (8x16字体, "HH:MM:SS" 8字符×8px=64px, 居中(128-64)/2=32)*/
#define CD_TIME_Y      22
static const int16_t cd_field_x[3] = {32, 56, 80};   /*HH/MM/SS 的 X 坐标*/
static const uint8_t cd_field_w[3] = {16, 16, 16};   /*每字段宽 (2位×8px)*/


/*重绘标志复用 watch.c 的 (通过 extern 无法直接访问)
 *实际上 TimeSetting 需要 watch.c 配合: Watch_Tick 的变化检测会置 watch_need_redraw
 *但 TimeSetting 内部的 flag 消费/闪烁也需要触发重绘, 用 IsTimeStateChanged 兜底
 */


static void cd_HandleKey(void)
{
	/*── Cd_Finished: 任意键确认 → 回 Idle (清空设定) ──*/
	if (Cd_State == Cd_Finished)
	{
		if (Key_IsPressed(KEY_UP) || Key_IsPressed(KEY_DOWN)
		 || Key_IsPressed(KEY_OK) || Key_IsPressed(KEY_BACK))
		{
			Cd_State    = Cd_Idle;
			Cd_Selected = Hour;
			showtime[0] = showtime[1] = showtime[2] = 0;
			past_ms = 0;
			begin_ms = 0;
			remaintime_ms = 0;
			cd_redraw = 1;
		}
		return;              /*Finished 态只响应"任意键确认", 不走下面的逻辑*/
	}

	if(Key_IsPressed(KEY_UP))
	{
		if(Cd_State==Cd_Idle)
		{
			Cd_Selected++;
			if(Cd_Selected>4)
				Cd_Selected=1;
		}
		else if(Cd_State==Editing)
		{
			TimeChangeFlag=1;
		}
		cd_redraw = 1;
	}
	else if(Key_IsPressed(KEY_DOWN))
	{
		if(Cd_State==Cd_Idle)
		{
			Cd_Selected--;
			if(Cd_Selected<1)
				Cd_Selected=4;
		}
		else if(Cd_State==Editing)
		{
			TimeChangeFlag=2;
		}
		if(Cd_State==Cd_Pause)
		{
			Cd_State=Cd_Idle;
			past_ms = 0;         // 复位累计
			begin_ms = 0;
			remaintime_ms = 0;
		}
		cd_redraw = 1;
	}
	else if(Key_IsPressed(KEY_OK))
	{
		if(Cd_State==Cd_Idle&&Cd_Selected!=Cd_Start)
		{
			Cd_State=Editing;
		}
		else if(Cd_State==Editing)
		{
			Cd_State=Cd_Idle;
			Cd_Selected++;
			if(Cd_Selected>3)
				Cd_Selected=1;
		}
		else if(Cd_State==Cd_Idle&&Cd_Selected==Cd_Start)
		{
			Cd_State=Cd_Running;
			set_ms=(showtime[0]*3600+showtime[1]*60+showtime[2])*1000;
			begin_ms=Timer_GetTick();
		}
		else if(Cd_State==Cd_Pause)
		{
			Cd_State=Cd_Running;
			begin_ms=Timer_GetTick();
		}
		else if(Cd_State==Cd_Running)
		{
			Cd_State=Cd_Pause;
			past_ms+=Timer_GetTick()-begin_ms;
		}
		cd_redraw = 1;
	}
	else if(Key_IsPressed(KEY_BACK))
	{
		Cd_Selected=Hour;
		Cd_State=Cd_Idle;
		set_ms=0;
		past_ms = 0;
		begin_ms = 0;
		remaintime_ms = 0;
		showtime[Hour-1]=0;
		showtime[Minute-1]=0;
		showtime[Second-1]=0;
		IsFirstEnter= 1;
		Watch_ExitLeafPage();
		return;
	}
}


/*══════════════════════════════════════════════════════════════
 *  显示绘制 (私有)
 *
 *  布局 (128x64):
 *    Y=0~15   标题 "Countdown" (反相高亮)
 *    Y=22~37  时间 "HH:MM:SS"  (8x16, 居中)
 *             - Idle:     选中字段反相框; 选 Start 时下方画 Start 按钮
 *             - Editing:  当前字段闪烁 (灭半周跳过)
 *             - Running:  正常显示剩余时间
 *             - Pause:    整个时间闪烁
 *             - Finished: 00:00:00 + "Time Up!" 闪烁
 *    Y=41~49  Start 按钮 (仅 Idle+选中Start 时显示, 反相高亮)
 *    Y=54~61  状态/操作提示 (6x8)
 *══════════════════════════════════════════════════════════════*/
static void Cd_Display(void)
{
	uint8_t hh, mm, ss;
	uint8_t edit_idx;        /*Editing 态正在编辑的字段索引 (0/1/2), 255=无*/

	cd_redraw = 0;
	OLED_Clear();

	/*── 标题 "Countdown" (9字符×8px=72, 居中(128-72)/2=28) ──*/
	OLED_ShowString(28, 0, "Countdown", OLED_8X16);
	OLED_ReverseArea(0, 0, 128, 16);

	/*── 决定显示哪个时间 ──*/
	if (Cd_State == Cd_Running || Cd_State == Cd_Pause || Cd_State == Cd_Finished)
	{
		/*运行/暂停/到时: 显示剩余时间 (从 remaintime_ms 算)*/
		hh = (uint8_t)(remaintime_ms / 3600000UL);
		mm = (uint8_t)((remaintime_ms / 60000UL) % 60);
		ss = (uint8_t)((remaintime_ms / 1000UL)  % 60);
	}
	else
	{
		/*Idle/Editing: 显示用户设定的 showtime*/
		hh = (uint8_t)showtime[0];
		mm = (uint8_t)showtime[1];
		ss = (uint8_t)showtime[2];
	}

	/*── 画时间 HH:MM:SS ──
	 * 闪烁跳过规则:
	 *   Pause/Finished + 灭半周 → 整个时间不画
	 *   Editing + 当前字段 + 灭半周 → 该字段不画
	 */
	edit_idx = (Cd_State == Editing) ? (uint8_t)(Cd_Selected - 1) : 255;

	if (!(Cd_State == Cd_Pause    && cd_blink == 0)
	 && !(Cd_State == Cd_Finished && cd_blink == 0))
	{
		/*Hour*/
		if (!(Cd_State == Editing && edit_idx == 0 && cd_blink == 0))
			OLED_ShowNum(cd_field_x[0], CD_TIME_Y, hh, 2, OLED_8X16);
		OLED_ShowChar(cd_field_x[0] + 16, CD_TIME_Y, ':', OLED_8X16);

		/*Minute*/
		if (!(Cd_State == Editing && edit_idx == 1 && cd_blink == 0))
			OLED_ShowNum(cd_field_x[1], CD_TIME_Y, mm, 2, OLED_8X16);
		OLED_ShowChar(cd_field_x[1] + 16, CD_TIME_Y, ':', OLED_8X16);

		/*Second*/
		if (!(Cd_State == Editing && edit_idx == 2 && cd_blink == 0))
			OLED_ShowNum(cd_field_x[2], CD_TIME_Y, ss, 2, OLED_8X16);
	}

	/*── Idle 态: 选中 H/M/S 时画反相框 (选中提示) ──*/
	if (Cd_State == Cd_Idle && Cd_Selected >= Hour && Cd_Selected <= Second)
	{
		uint8_t ci = Cd_Selected - 1;
		OLED_ReverseArea(cd_field_x[ci], CD_TIME_Y, cd_field_w[ci], 16);
	}

	/*── Idle 态: 选中 Start 时画 "Start" 按钮 (反相高亮) ──*/
	if (Cd_State == Cd_Idle)
	{
		/*"Start" 5字符×6px=30, 居中(128-30)/2=49*/
		OLED_ShowString(49, 42, "Start", OLED_6X8);
		if(Cd_Selected == Cd_Start)
		 OLED_ReverseArea(47, 41, 34, 9);
	}

	/*── 底部状态/操作提示 (6x8) ──*/
	switch (Cd_State)
	{
		case Cd_Idle:
			if (Cd_Selected == Cd_Start)
				OLED_ShowString(16, 54, "OK:Start BK:Exit", OLED_6X8);
			else
				OLED_ShowString(16, 54, "OK:Edit  BK:Exit", OLED_6X8);
			break;
		case Editing:
			OLED_ShowString(16, 54, "UP/DN    OK:Next", OLED_6X8);
			break;
		case Cd_Running:
			OLED_ShowString(40, 54, "OK:Pause", OLED_6X8);
			break;
		case Cd_Pause:
			OLED_ShowString(4, 54, "OK:Go DN:Rst BK:Exit", OLED_6X8);
			break;
		case Cd_Finished:
			if (cd_blink)
				OLED_ShowString(43, 54, "Time Up!", OLED_6X8);
			break;
	}

	OLED_Update();
}



void Countdown_Tick(void)
{
	if (NowState != Countdown)
	{
		return;
	}
	if (IsFirstEnter)
	{
		Cd_Display();
		IsFirstEnter=0;
	}
	cd_HandleKey();
	if(Cd_State==Editing)
	{
		if(TimeChangeFlag==1)
		{
			showtime[Cd_Selected-1]++;
			TimeChangeFlag=0;
		}else if(TimeChangeFlag==2)
		{
			showtime[Cd_Selected-1]--;
			TimeChangeFlag=0;
		}
		if(showtime[0]>24)
			showtime[0]=0;
		else if(showtime[0]<0)
			showtime[0]=24;
		if(showtime[1]>60)
			showtime[1]=0;
		else if(showtime[1]<0)
			showtime[1]=60;
		if(showtime[2]>60)
			showtime[2]=0;	
		else if(showtime[2]<0)
			showtime[2]=60;
	}
	else if(Cd_State==Cd_Running)
	{
		/*用有符号算, 防止 uint32_t 减法下溢成 42 亿*/
		int32_t remain = (int32_t)set_ms - (int32_t)past_ms
		               - (int32_t)(Timer_GetTick() - begin_ms);
		if (remain <= 0)
		{
			/*到时: 进 Finished 态, 显示 00:00:00 + Time Up! 闪烁*/
			remaintime_ms = 0;
			Cd_State      = Cd_Finished;
			cd_blink      = 1;
			cd_blink_cnt  = 0;
		}
		else
		{
			remaintime_ms = (uint32_t)remain;
		}
		cd_redraw = 1;
	}

	/*══ 闪烁定时 (Editing/Pause/Finished 态, 0.5秒翻转一次) ══*/
	if (Cd_State == Editing || Cd_State == Cd_Pause || Cd_State == Cd_Finished)
	{
		if (cd_blink_cnt < 50) cd_blink_cnt++;
		else
		{
			cd_blink = !cd_blink;
			cd_blink_cnt = 0;
			cd_redraw = 1;
		}
	}
	else
	{
		/*非闪烁态强制显示, 避免从闪烁态退出时停在灭半周*/
		cd_blink = 1;
	}
	if(cd_redraw)
	{
		Cd_Display();
		cd_redraw=0;
	}
}
