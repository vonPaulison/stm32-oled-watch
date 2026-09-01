/**
 * ============================================================
 *  Timer.c — 系统毫秒节拍 (TIM2 硬件中断驱动)
 *
 *  原理:
 *    TIM2 配置成 1ms 周期中断, 每次中断 g_tick++
 *    主循环通过 Timer_GetTick() 读 g_tick, 算时间差
 *
 *  为什么不用 SysTick?
 *    SysTick 已被 Delay.c 占用 (阻塞延时), 会被反复重配, 不能复用
 *
 *  为什么不用 RTC?
 *    RTC 是 1Hz (秒级), 精度不够秒表 (要 ms 级)
 *
 *  时钟配置:
 *    TIM2 挂在 APB1, PCLK1=36MHz
 *    但 APB1 定时器时钟自动 ×2 = 72MHz
 *    预分频 72-1 → 计数频率 1MHz (1us)
 *    重装值 1000-1 → 1ms 溢出一次
 * ============================================================
 */

#include "stm32f10x.h"
#include "Timer.h"
#include "Key.h"             /*Key_Scan() — 移到中断里调用, 不受主循环拖慢*/

/*══════════════════════════════════════════════════════════════
 *  全局毫秒计数器 (volatile!)
 *  volatile 关键字告诉编译器: 这个变量可能被中断修改,
 *  每次用都要重新从内存读, 不要优化到寄存器里缓存
 *  不加 volatile 会导致主循环读到旧值, 计时不准
 *══════════════════════════════════════════════════════════════*/
static volatile uint32_t g_tick = 0;

/*按键扫描分频计数器 (每10次中断=10ms调一次Key_Scan)*/
static volatile uint8_t key_scan_divider = 0;


/**
 * TIM2 初始化 — 配置成 1ms 周期中断
 *
 * 调用时机: main() 里, 进入 while(1) 之前
 *           优先级要在 OLED_Init / Watch_Init 之后
 */
void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*开启 TIM2 时钟 (挂在 APB1)*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /*定时器基础配置
     * TIM_ClockDivision: 时钟分频, 采样用, 这里不影响计时
     * TIM_CounterMode:   向上计数
     * TIM_Prescaler:     72-1 → 计数频率 = 72MHz/72 = 1MHz (1us)
     * TIM_Period:        1000-1 → 每 1000 次 (1ms) 溢出一次
     */
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Prescaler         = 72 - 1;
    TIM_TimeBaseStructure.TIM_Period            = 1000 - 1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /*清除中断挂起标志 (防初始化时误触发一次中断)*/
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    /*使能 TIM2 更新中断*/
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /*配置 NVIC (嵌套向量中断控制器)
     * 优先级: 抢占 1, 子优先级 0
     *   比 Key_Scan (主循环轮询, 非中断) 高
     *   比 HardFault (优先级 -1) 低
     * 使能 TIM2 中断通道
     */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*启动 TIM2*/
    TIM_Cmd(TIM2, ENABLE);
}


/**
 * 获取系统毫秒计数
 * @return 系统启动以来的毫秒数
 */
uint32_t Timer_GetTick(void)
{
    return g_tick;
}


/*══════════════════════════════════════════════════════════════
 *  TIM2 中断服务函数
 *
 *  每 1ms 触发一次, CPU 自动跳到这里执行
 *  执行完自动返回主循环被打断的位置 (硬件保证, 不用手动 return)
 *
 *  函数名固定为 TIM2_IRQHandler (与启动文件里的中断向量表对应)
 *  不能改名, 不能加参数, 不能有返回值
 *══════════════════════════════════════════════════════════════*/
void TIM2_IRQHandler(void)
{
    /*检查是不是 TIM2 的更新中断 (计数溢出)*/
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        g_tick++;                                   /*毫秒计数 +1*/

        /*每 10ms 调一次 Key_Scan (中断里扫描按键)
         *原来 Key_Scan 在主循环里调, 但 OLED_Update 耗时 ~15ms
         *导致 Key_Scan 实际间隔变成 25ms, 消抖变慢 (125ms), 按键有延迟感
         *现在放进中断, 间隔固定 10ms, 消抖恢复 50ms, 按键立即响应
         */
        if (key_scan_divider < 9)
            key_scan_divider++;
        else
        {
            key_scan_divider = 0;
            Key_Scan();                             /*扫描4键状态机*/
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); /*清中断标志 (必须!)*/
                                                    /*不清的话会立刻再次触发, 死循环*/
    }
}
