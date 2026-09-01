#ifndef __WATCH_H
#define __WATCH_H
#define MAX_MENU_NODES 20

enum MenuIndex{
	Maininterface=1,
	Menu,
	Setting,
	 Time,
	 Light,
	About,
	Stopwatch,
	Countdown,
	FlashLight,
	 Normal,
	 Glitter,
	Test,
	 Compass,       /*指南针页面 (QMC5883P 三轴磁传感器)*/
	 TempHum,       /*温湿度页面 (AHT20 温湿度传感器)*/
	 indexcount
};

typedef struct{
	int lastnode;
	int nextnode[10];
	char *name;
}Info;

/*══════════════════════════════════════════════════════════════
 *  跨文件共享的全局变量 (extern 声明)
 *───────────────────────────────────────────────────────────────────
 *  定义在 watch.c, 其他模块(Stopwatch.c 等)通过 extern 访问
 *  原则: 只暴露"必须共享"的, 能 static 的绝不放这里
 *══════════════════════════════════════════════════════════════*/
extern char NowState;            /*当前菜单状态 (Maininterface/Menu/.../Stopwatch)*/


/*══════════════════════════════════════════════════════════════
 *  公共函数声明
 *══════════════════════════════════════════════════════════════*/

/**
 * 手表系统初始化
 * - 初始化4键 GPIO 与状态机
 * - 重置菜单状态到主界面
 * - 标记首帧需要重绘
 * 调用时机: OLED_Init() 之后, 进入主循环之前
 */
void Watch_Init(void);

/**
 * 手表主节拍 (必须每 10ms 调用一次)
 * - 扫描按键 → 迁移菜单状态 → 路由绘制画面
 * - 内部自动判断是否需要重绘 (状态/光标变化时才重画, 防闪屏)
 * - 主界面每秒定时刷新一次 (走时显示)
 * 调用约定: 由 main 的 Delay_ms(10) 保证 10ms 节拍
 */
void Watch_Tick(void);

/**
滚动菜单显示函数
 */
void MenuSelect_Display(void);

/**
 * 退出当前叶子页面, 回到上级菜单
 * - 供 Stopwatch.c / TimeSetting.c 等叶子模块调用
 * - 内部封装: 改 NowState = 父节点, selectedNext=0, ShowStart=0
 * - 避免叶子模块直接碰 watch.c 的私有变量 (封装原则)
 */
void Watch_ExitLeafPage(void);

#endif
