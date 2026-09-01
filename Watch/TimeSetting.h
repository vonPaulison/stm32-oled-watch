#ifndef __TIMESETTING_H
#define __TIMESETTING_H

/**
 * 时间设置模块接口
 *
 * 由 Watch_Tick 在 NowState==Time 时调用, 全权处理:
 *   字段选择(UP/DOWN) + 字段编辑(OK进入) + 闪烁提示 + 显示绘制
 *
 * 完全自治: 内部管理重绘标志和闪烁状态, 不依赖 watch.c 的私有变量
 */
void TimeSetting_Tick(void);

#endif
