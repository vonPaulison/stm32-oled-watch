#ifndef __LIGHTSETTING_H
#define __LIGHTSETTING_H

/**
 * 亮度调节模块接口
 *
 * 由 Watch_Tick 在 NowState==Light 时调用, 全权处理:
 *   UP/DOWN 调节亮度档位 + 实时生效 + 条形图绘制
 *
 * 完全自治: 内部管理重绘标志, 不依赖 watch.c 的私有变量
 */
void LightSetting_Tick(void);

#endif
