#include <stdio.h>
#include "OLED.h"
#include "OLED_Data.h"
#include "watch.h"
#include "Key.h"
#include <string.h>
#include "MyRTC.h"
#include "Stopwatch.h"       /*Stopwatch_Tick() — 秒表模块*/
#include "TimeSetting.h"     /*TimeSetting_Tick() — 时间设置模块*/
#include "flashlight.h"      /*Flashlight_Tick() — 手电筒模块*/
#include "LightSetting.h"    /*LightSetting_Tick() — 亮度调节模块*/
#include "Countdown.h"       /*Countdown_Tick() — 倒计时模块*/
#include "Compass.h"        /*Compass_Tick() — 指南针模块 (I2C与OLED共用PB8/PB9)*/
#include "TempHum.h"        /*TempHum_Tick() — 温湿度模块 (AHT20, 同一条I2C总线)*/


/*══════════════════════════════════════════════════════════════
 *  滚动菜单参数
 *  屏幕 64px 高, 标题栏 0~15 占用 16px, 剩余 48px 可用
 *  每项 12px (8px字体 + 4px间距) → 最多显示 4 项
 *══════════════════════════════════════════════════════════════*/
#define MENU_VISIBLE_COUNT  4    /* 可视区最多显示项数 */
#define MENU_ITEM_PITCH     12   /* 每项行距 (8px字体 + 4px空隙) */
#define MENU_ITEM_START_Y   17   /* 首项起始Y (16px标题栏 + 1px留白) */

char NowState=Maininterface;
static int selectedNext=0;          //
static uint8_t ShowStart = 0;    /* 滚动偏移: 可视区第一项的索引 */

/*══════════════════════════════════════════════════════════════
 *  每个菜单节点的光标/滚动位置记忆
 *  进入子菜单前保存当前节点位置, 退出时恢复
 *  实现"BACK返回后光标停在原来选的那项"
 *  初值全0: 首次进入任何菜单都从第0项开始
 *══════════════════════════════════════════════════════════════*/
static int8_t  menu_cursor[indexcount] = {0};
static uint8_t menu_scroll[indexcount] = {0};

//
const Info maininterface = {Maininterface, {Menu}, "Maininterface"};
const Info menu = {Maininterface, {Stopwatch,Countdown,FlashLight,Setting,About,Test,Compass,TempHum}, "Menu"};
const Info setting = {Menu, {Time,Light}, "Setting"};
const Info time = {Setting, {Time}, "Time"};
const Info light = {Setting, {Light}, "Light"};
const Info about = {Menu, {About}, "About"};
const Info stopwatch = {Menu, {Stopwatch}, "Stopwatch"};
const Info countdown = {Menu, {Countdown}, "Countdown"};
const Info flashLight = {Menu, {Normal,Glitter}, "FlashLight"};
const Info normal = {FlashLight, {Normal}, "Normal"};
const Info glitter = {FlashLight, {Glitter}, "Glitter"};
const Info test = {Menu, {Test}, "Test"};
const Info compass = {Menu, {Compass}, "Compass"};
const Info temphum = {Menu, {TempHum}, "TempHum"};


const Info* menuTable[MAX_MENU_NODES] = {
    [Maininterface] = &maininterface,
    [Menu]          = &menu,
    [Setting]       = &setting,
     [Time]         = &time, 
     [Light]        = &light,
    [About]         = &about,
	[Stopwatch]     = &stopwatch,
	[Countdown]     = &countdown,
	[FlashLight]    = &flashLight,
	 [Normal]    	= &normal,
	 [Glitter]      = &glitter,
	[Test]          = &test,
	[Compass]       = &compass,
	[TempHum]       = &temphum,
};


void Menu_HandleKey(void) //一般界面状态函数
{
    const Info* node = menuTable[NowState];
	if (node == NULL) return;
	/*叶子节点(Time/Stopwatch/FlashLight/Test)自己处理按键, 菜单状态机不干预
	 *判据: 叶子的 nextnode[0] 指向自己 → 与 NowState 相等
	 *这样以后加任何叶子节点都不用改这里
	 */
	if (node->nextnode[0] == NowState) return;

	char menumaxnextnode=0;
	char count;
	for(count=0;count<10;count++)
	{
		if(node->nextnode[count]!=0)
		{
			menumaxnextnode++;
		}
	}
    if(Key_IsPressed(KEY_UP))
	{
		selectedNext--;
		if(selectedNext<0)          /*有上界保护, 不会变成-1*/
		{
			selectedNext=menumaxnextnode-1;
		}
	}
    else if(Key_IsPressed(KEY_DOWN))
	{
		selectedNext++;
		if(selectedNext>=menumaxnextnode)
		{
			selectedNext=0;
		}
	}
    else if(Key_IsPressed(KEY_OK))
	{
		if(NowState==node->nextnode[0])
		return;
		if (selectedNext < menumaxnextnode && node->nextnode[selectedNext] != 0)
		{
			/*保存当前菜单的光标和滚动位置, 退出时恢复*/
			menu_cursor[NowState] = selectedNext;
			menu_scroll[NowState] = ShowStart;
			/*切换到子菜单, 恢复它上次的光标位置 (首次进入为初值0)*/
			NowState = node->nextnode[selectedNext];
			selectedNext = 0;
			ShowStart    = 0;
        }
	}
    else if(Key_IsPressed(KEY_BACK))
	{
		 /*切换到父菜单, 恢复它上次的光标位置*/
		 NowState = node->lastnode;
		 selectedNext = menu_cursor[NowState];
		 ShowStart    = menu_scroll[NowState];
	}

	/*══ 滚动调整: 让选中项始终保持在可视区内 ══
	 * 选中项 < 可视区顶部 → 可视区上移跟上
	 * 选中项 >= 可视区底部 → 可视区下移
	 * (现在menu只有4项恰好放得下, 滚动不触发; 加项后会自动滚动)
	 */
	if (selectedNext < ShowStart)
		ShowStart = selectedNext;
	else if (selectedNext >= ShowStart + MENU_VISIBLE_COUNT)
		ShowStart = selectedNext - MENU_VISIBLE_COUNT + 1;
}

void MenuSelect_Display(void)//有子菜单的页面绘制 (含滚动)
{
    const Info* node = menuTable[NowState];
    uint8_t i;
    uint8_t total_children;    /* 实际子节点总数 */
    uint8_t item_y;            /* 当前绘制项Y坐标 */
    uint8_t draw_count;        /* 本次实际绘制项数 */

    if (node == NULL) return;

    /* 统计子节点总数 */
    total_children = 0;
    for (i = 0; i < 10 && node->nextnode[i] != 0; i++)
        total_children++;

    OLED_Clear();
    OLED_ShowString((128 - strlen(node->name) * 8) / 2, 0, node->name, OLED_8X16);  /* 标题 */
    OLED_ReverseArea(0, 0, 128, 16);

    if (NowState != node->nextnode[0])
    {
        /* 本次要绘制的项数 = min(可视区容量, 剩余可选项数) */
        draw_count = MENU_VISIBLE_COUNT;
        if (ShowStart + draw_count > total_children)
            draw_count = total_children - ShowStart;
        for (i = 0; i < draw_count; i++)
        {
            const Info* child = menuTable[node->nextnode[ShowStart + i]];

            /* 计算该项Y坐标 (首项17, 每项间距12px) */
            item_y = MENU_ITEM_START_Y + i * MENU_ITEM_PITCH;

            /* 光标指示符 (选中项才画">") */
            if ((ShowStart + i) == (uint8_t)selectedNext)
                OLED_ShowString((128 - strlen(child->name) * 6) / 2 - 8, item_y, ">", OLED_6X8);

            /* 菜单项名称 */
            if (child)
                OLED_ShowString((128 - strlen(child->name) * 6) / 2, item_y, child->name, OLED_6X8);
        }
        OLED_Update();
    }
}

void SpecialMenu_Display(void) // 主界面显示时间
{
    if (NowState == Maininterface)
	{
		OLED_ShowString(20,0,maininterface.name,OLED_6X8);
		OLED_ShowString(18, 9, "XXXX-XX-XX",OLED_8X16);
		OLED_ShowString(26, 26, "XX:XX:XX",OLED_8X16);
		MyRTC_ReadTime();										//RTC读取时间，最新的时间存储到MyRTC_Time数组中
		OLED_ShowNum(18, 9, MyRTC_Time[0], 4,OLED_8X16);		//显示MyRTC_Time数组中的时间值，年
		OLED_ShowNum(58, 9, MyRTC_Time[1], 2,OLED_8X16);		//月
		OLED_ShowNum(82, 9, MyRTC_Time[2], 2,OLED_8X16);		//日
		OLED_ShowNum(26, 26, MyRTC_Time[3], 2,OLED_8X16);		//时
		OLED_ShowNum(50, 26, MyRTC_Time[4], 2,OLED_8X16);		//分
		OLED_ShowNum(74, 26, MyRTC_Time[5], 2,OLED_8X16);		//秒
		return;
	}
	if (NowState == About)
	{
		OLED_ShowString(47,0,about.name,OLED_6X8);
		OLED_ShowString(0,9,"This project is prese",OLED_6X8);
		OLED_ShowString(0,18,"-ented to my dear fri",OLED_6X8);
		OLED_ShowString(0,27,"-end 'ZhangXiaoChuan'",OLED_6X8);
		OLED_ShowString(0,36,"to appreciate him for",OLED_6X8);
		OLED_ShowString(0,45,"his encouragement",OLED_6X8);
		OLED_ShowString(0,54,"Von Ziegenburg 2026.7",OLED_6X8);
		return;
	}
	if (NowState == Test)
	{
		OLED_ShowString(0,28,"This page is for test",OLED_6X8);
		
		return;
	}

}

static uint8_t  watch_need_redraw = 1;    
static int      watch_last_state  = -1;    /*上一帧的菜单状态 (初始-1, 强制首帧检测到"变化")*/
static int8_t   watch_last_select = -1;    /*同上*/
static uint8_t  watch_cnt = 0 ;

static uint8_t Watch_HasChildren(const Info* node)
{
	/*防御性判空 + 双重条件判断*/
	return (node != NULL)
	    && (node->nextnode[0] != 0)
	    && (node->nextnode[0] != NowState);
}

void Watch_Init(void)
{
	Key_Init();                   /*初始化按键: GPIO配置 + 状态机清零*/
	NowState = Maininterface;	  /*开机进入主界面*/
	selectedNext  = 0;            /*菜单选中项归零*/
	watch_need_redraw = 1;        /*标记需要绘制首帧*/
	watch_last_state  = -1;       /*置为非法值, 强制首帧检测到"状态变化"*/
	watch_last_select = -1;
	watch_cnt = 0 ;
}

/**
 * 退出当前叶子页面, 回到上级菜单
 * 供 Stopwatch.c / TimeSetting.c / flashlight.c 等叶子模块调用
 * 封装了"切 NowState + 恢复父菜单光标"两件事
 * 避免叶子模块直接访问 watch.c 的 static 变量
 */
void Watch_ExitLeafPage(void)
{
	const Info* node = menuTable[NowState];
	if (node == NULL) return;
	/*叶子页面没有"下次进入恢复"的需求, 不保存它的光标*/
	NowState     = node->lastnode;   /*回到父节点*/
	selectedNext = menu_cursor[NowState];   /*恢复父菜单的光标位置*/
	ShowStart    = menu_scroll[NowState];
}

void Watch_Tick(void)
{
	watch_cnt++;
	const Info* node;
	/*Key_Scan 已移到 TIM2 中断 (Timer.c) 里每 10ms 调用
	 *不再在主循环调, 这样 OLED 拖慢主循环也不会影响按键消抖
	 *主循环只负责: 读按键标志 → 状态迁移 → 画面路由
	 */

	/*══ 状态切换时清所有按键 flag (防幽灵事件) ══
	 *锁存模式下, 未被读取的 flag 会一直留着
	 *比如在 Stopwatch 页按 UP (Stopwatch 不查UP), flag 残留
	 *回到 Menu 时 Menu 查 UP 会读到1, 光标幽灵上移
	 *解决: NowState 变化时, 把4个键的 flag 全读一遍 (读即清)
	 */
	if (NowState != watch_last_state)
	{
		Key_IsPressed(KEY_UP);
		Key_IsPressed(KEY_DOWN);
		Key_IsPressed(KEY_OK);
		Key_IsPressed(KEY_BACK);
		/*长按键 flag 也要一并清: long_flag 同样是锁存到读
		 *例: 在秒表页长按 OK (秒表只查 Key_IsPressed 不查长按),
		 *flag 残留 → 之后进指南针页, Compass 查 Key_IsLongPress(KEY_OK)
		 *会读到残留 flag, 莫名其妙进入校准模式 —— 幽灵长按
		 */
		Key_IsLongPress(KEY_UP);
		Key_IsLongPress(KEY_DOWN);
		Key_IsLongPress(KEY_OK);
		Key_IsLongPress(KEY_BACK);
	}

	Menu_HandleKey();
	if (NowState != watch_last_state || selectedNext != watch_last_select)
	{
		watch_need_redraw = 1;             /*状态或光标变了, 需要重画*/
		watch_last_state  = NowState;      /*记录本帧状态, 供下帧比较*/
		watch_last_select = selectedNext;
	}
	/*══ 第4步: 画面路由 ══
	 * 根据 NowState 分发到对应的绘制函数
	 */
	node = menuTable[NowState];
	if (node == NULL) return;
	if (NowState == Maininterface)
	{	 
		if(watch_cnt>=98)
		{
			watch_need_redraw=1;
			watch_cnt=0;
		}
		if (watch_need_redraw)
		{
			OLED_Clear();                 
			SpecialMenu_Display();       
			OLED_Update();
			watch_need_redraw = 0;  
		}
	}
	else if (Watch_HasChildren(node))
	{

		if (watch_need_redraw)
		{
			MenuSelect_Display();
			watch_need_redraw = 0;
		}
	}
	else
	{
		if (NowState == Time)
		{
			/*Time页面交给时间编辑器自治处理 (按键+闪烁+重绘)*/
			TimeSetting_Tick();
		}
		else if (NowState == Stopwatch)
		{
			/*Stopwatch页面交给秒表模块自治处理 (按键+计时+闪烁+重绘)*/
			Stopwatch_Tick();
		}
		else if (NowState == Countdown)
		{
			/*倒计时页, 交给 Countdown.c 自治处理*/
			Countdown_Tick();
		}
		else if (NowState == Compass)
		{
			/*指南针页, 交给 Compass.c 自治处理 (含传感器懒初始化与校准)*/
			Compass_Tick();
		}
		else if (NowState == TempHum)
		{
			/*温湿度页, 交给 TempHum.c 自治处理 (含传感器懒初始化)*/
			TempHum_Tick();
		}
		else if (NowState == Normal || NowState == Glitter)
		{
			/*手电筒常亮/闪烁模式, 交给 flashlight.c 自治处理*/
			Flashlight_Tick();
		}
		else if (NowState == Light)
		{
			/*亮度调节页, 交给 LightSetting.c 自治处理*/
			LightSetting_Tick();
		}
		else
		{
			/*简单叶子页面 (About/Test)
			 *这些页面没有自己的Tick, 在这里处理BACK退出
			 *Time/Stopwatch/FlashLight/Light 有自己的Tick自治, 不走这个分支
			 */
			if (Key_IsPressed(KEY_BACK))
			{
				Watch_ExitLeafPage();
				return;              /*NowState已变, 不再绘制*/
			}
			if (watch_need_redraw)
			{
				OLED_Clear();
				SpecialMenu_Display();
				OLED_Update();
				watch_need_redraw = 0;
			}
		}
	}
}
