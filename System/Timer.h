#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

/**
 * 系统毫秒节拍模块 (基于 TIM2 硬件中断)
 *
 * 提供 Timer_GetTick() 返回系统启动以来的毫秒数
 * 精度: 1ms, 不受主循环阻塞影响
 *
 * 用法:
 *   Timer_Init();              // main 里初始化
 *   uint32_t t0 = Timer_GetTick();
 *   ... 做点事 ...
 *   uint32_t elapsed = Timer_GetTick() - t0;   // 真实经过的 ms
 */
void Timer_Init(void);

/**
 * 获取系统启动以来的毫秒计数
 * @return 毫秒数 (uint32_t, 约 49.7 天溢出回 0)
 */
uint32_t Timer_GetTick(void);

#endif
