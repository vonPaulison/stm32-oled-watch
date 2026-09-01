#ifndef __COUNTDOWN_H
#define __COUNTDOWN_H

/**
 * 倒计时模块接口
 *
 * 由 Watch_Tick 在 NowState==countdown 时调用, 全权处理:
 *   字段选择(UP/DOWN) + 字段编辑(OK进入) + 闪烁提示 + 显示绘制
 *
 * 完全自治
 */
void Countdown_Tick(void);

#endif
