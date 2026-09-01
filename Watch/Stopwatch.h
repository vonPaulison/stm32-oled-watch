#ifndef __STOPWATCH_H
#define __STOPWATCH_H

/**
 * 秒表模块接口
 *
 * 由 Watch_Tick 在 NowState==Stopwatch 时调用, 全权处理:
 *   按键扫描(Start/Stop/Reset/退出) + 计时累加 + 显示绘制
 *
 * 完全自治: 内部管理重绘标志, 不依赖 watch.c 的 watch_need_redraw
 */
void Stopwatch_Tick(void);

#endif
