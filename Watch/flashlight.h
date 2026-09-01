#ifndef __FLASHLIGHT_H
#define __FLASHLIGHT_H

/**
 * 手电筒模块接口
 *
 * 由 Watch_Tick 在 NowState==FlashLight 时调用, 全权处理:
 *   OK 开灯 / BACK 关灯并退出 + 画面绘制
 *
 * 完全自治: 内部管理重绘标志和开关状态, 不依赖 watch.c 的私有变量
 */
void Flashlight_Tick(void);

#endif
