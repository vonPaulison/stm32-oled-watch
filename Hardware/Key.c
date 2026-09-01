/**
 * ============================================================
 *  Key.c — 4键输入驱动实现 (状态机 + 消抖 + 长按)
 *
 *  硬件连接:
 *    PA0 ─── 按键 UP   ─── GND  (按下=低电平)
 *    PA1 ─── 按键 DOWN ─── GND
 *    PA2 ─── 按键 OK   ─── GND
 *    PA3 ─── 按键 BACK ─── GND
 *    GPIO 配置为内部上拉 (IPU)，未按时读到高电平(1)，按下时读到低电平(0)
 *
 *  设计要点:
 *    1. 每个按键独立维护一个状态机，互不干扰
 *    2. 消抖: 连续5次(50ms)读到低电平才确认按下
 *    3. 长按: 按住超过800ms触发长按事件(单次边沿)
 *    4. 边沿触发 + 锁存: press_flag/long_flag 仅在状态转换时置1,
 *       Key_Scan 不清零 (锁存), 由 Key_IsPressed/Key_IsLongPress
 *       读取时清零 (读后即清)。这样中断设的事件不会因主循环慢而丢失
 *    5. 电平触发: Key_IsHeld() 直接读引脚，每次调用反映实时状态
 *
 *  状态机详解:
 *  ┌─────────────────────────────────────────────────────────┐
 *  │  IDLE (空闲)                                            │
 *  │    raw=1 → 保持 IDLE                                   │
 *  │    raw=0 → 进入 DEBOUNCE，hold_cnt清零                 │
 *  ├─────────────────────────────────────────────────────────┤
 *  │  DEBOUNCE (消抖确认)                                   │
 *  │    raw=0 → hold_cnt++，若>=5 → 进入 PRESSED，置press   │
 *  │    raw=1 → 抖动，退回 IDLE                             │
 *  ├─────────────────────────────────────────────────────────┤
 *  │  PRESSED (已按下，等待松开或长按)                      │
 *  │    raw=0 → hold_cnt++，若>=80(800ms) → 进入LONG，置long│
 *  │    raw=1 → 短按结束，退回 IDLE                         │
 *  ├─────────────────────────────────────────────────────────┤
 *  │  LONG (长按已触发，等待松开)                           │
 *  │    raw=0 → 保持 LONG (防止重复触发long_flag)           │
 *  │    raw=1 → 松开，退回 IDLE                             │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  调用周期: Key_Scan() 必须每 10ms 调用一次
 *           (与主循环的 Delay_ms(10) 配合)
 * ============================================================
 */

#include "stm32f10x.h"       /*STM32F10x 标准外设库: GPIO读写、时钟控制*/
#include "Key.h"             /*本模块头文件: 按键ID、状态定义*/

/**
 * 单个按键的控制块
 * 每个按键独立维护一份，共 KEY_NUM=4 份
 */
typedef struct {
    GPIO_TypeDef* port;      /*GPIO端口 (GPIOA)*/
    uint16_t pin;            /*引脚号 (GPIO_Pin_0 ~ GPIO_Pin_3)*/
    uint8_t state;           /*当前状态: IDLE/DEBOUNCE/PRESSED/LONG*/
    uint16_t hold_cnt;       /*按住计数器 (单位: 10ms)，用于消抖和长按计时*/
    volatile uint8_t press_flag;  /*"刚按下"标志 (边沿，仅当次扫描有效)*/
                                   /*volatile: 中断写, 主循环读, 必须加*/
    volatile uint8_t long_flag;   /*"长按"标志 (边沿，仅当次扫描有效)*/
                                   /*volatile: 同上*/
} Key_t;

/**
 * 4个按键的控制块数组
 * static 限定: 仅本文件内可见
 * 索引对应: keys[KEY_UP], keys[KEY_DOWN], keys[KEY_OK], keys[KEY_BACK]
 */
static Key_t keys[KEY_NUM];

/**
 * 读取单个按键的原始电平
 * GPIO_ReadInputDataBit: 读引脚输入数据寄存器
 * Bit_RESET = 0 = 低电平 = 按键按下
 * Bit_SET   = 1 = 高电平 = 按键松开
 *
 * @param key_id  按键ID
 * @return 0=按下(低电平), 1=松开(高电平)
 */
static uint8_t Key_ReadPin(uint8_t key_id)
{
    if (GPIO_ReadInputDataBit(keys[key_id].port, keys[key_id].pin) == Bit_RESET)
        return 0;   /*按下*/
    return 1;       /*松开*/
}

/**
 * 按键初始化
 *
 * 硬件配置:
 *   GPIOA 时钟使能
 *   PA0~PA3 配置为:
 *     - 模式: GPIO_Mode_IPU (Input Pull-Up, 内部上拉输入)
 *     - 速度: 50MHz
 *     内部上拉意味着: 未按时引脚被拉高 → 读到1
 *                    按下时引脚接地 → 读到0
 */
void Key_Init(void)
{
    uint8_t i;                              /*循环变量 (C89: 块顶部声明)*/
    GPIO_InitTypeDef GPIO_InitStructure;    /*GPIO初始化结构体*/

    /*开启 GPIOA 外设时钟 (挂在 APB2 总线上)*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /*配置 PA0, PA1, PA2, PA3 为内部上拉输入*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           /*内部上拉输入*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       /*50MHz输出速度(输入模式可忽略)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  /*写入配置到GPIOA*/

    /*绑定每个按键到对应的 GPIO 引脚*/
    keys[KEY_UP].port   = GPIOA;
    keys[KEY_UP].pin    = GPIO_Pin_0;       /*PA0 → 上键*/
    keys[KEY_DOWN].port = GPIOA;
    keys[KEY_DOWN].pin  = GPIO_Pin_1;       /*PA1 → 下键*/
    keys[KEY_OK].port   = GPIOA;
    keys[KEY_OK].pin    = GPIO_Pin_2;       /*PA2 → 确认键*/
    keys[KEY_BACK].port = GPIOA;
    keys[KEY_BACK].pin  = GPIO_Pin_3;       /*PA3 → 返回键*/

    /*初始化所有按键的状态机: 空闲, 计数器清零, 标志清零*/
    for (i = 0; i < KEY_NUM; i++) {
        keys[i].state = KEY_STATE_IDLE;
        keys[i].hold_cnt = 0;
        keys[i].press_flag = 0;
        keys[i].long_flag = 0;
    }
}

/**
 * 按键扫描 — 驱动所有按键的状态机
 *
 * 必须每 KEY_SCAN_PERIOD_MS (10ms) 调用一次 (在 TIM2 中断里)
 *
 * 每次调用流程:
 *   遍历4个按键:
 *     1. 读取引脚原始电平 (raw)
 *     2. 根据当前状态和 raw 执行状态转换
 *     3. 在转换到 PRESSED 状态时置 press_flag
 *        在转换到 LONG 状态时置 long_flag
 *
 * 注意: press_flag/long_flag 不在此函数清零 (锁存模式)
 *       由 Key_IsPressed / Key_IsLongPress 读取时清零
 *       这样主循环慢(OLED 15ms)也不会丢按键事件
 */
void Key_Scan(void)
{
    uint8_t i;      /*循环变量 (C89: 块顶部声明)*/

    for (i = 0; i < KEY_NUM; i++) {
        uint8_t raw;        /*当前引脚原始电平: 0=按下, 1=松开*/

        raw = Key_ReadPin(i);

        /*不在此处清 press_flag/long_flag (锁存模式, 由读取方清零)*/

        /*根据当前状态执行状态转换*/
        switch (keys[i].state) {

        /*─────────────────────────────────────────────────────
         * 状态: IDLE (空闲)
         * 等待按键按下 (raw从1变0)
         *─────────────────────────────────────────────────────*/
        case KEY_STATE_IDLE:
            if (raw == 0) {                             /*检测到低电平*/
                keys[i].state = KEY_STATE_DEBOUNCE;     /*进入消抖阶段*/
                keys[i].hold_cnt = 0;                   /*重置消抖计数器*/
            }
            break;

        /*─────────────────────────────────────────────────────
         * 状态: DEBOUNCE (消抖确认)
         * 连续5次(50ms)读到低电平 → 确认按下
         * 中途读到高电平 → 视为抖动，退回 IDLE
         *─────────────────────────────────────────────────────*/
        case KEY_STATE_DEBOUNCE:
            if (raw == 0) {
                keys[i].hold_cnt++;                     /*计数+1 (每次10ms)*/
                if (keys[i].hold_cnt >= 5) {            /*持续50ms低电平*/
                    keys[i].state = KEY_STATE_PRESSED;  /*消抖通过，进入按下状态*/
                    keys[i].hold_cnt = 0;               /*计数器归零(后续用于长按计时)*/
                    keys[i].press_flag = 1;             /*触发"刚按下"事件*/
                }
            } else {
                keys[i].state = KEY_STATE_IDLE;         /*抖动，退回空闲*/
                keys[i].hold_cnt = 0;
            }
            break;

        /*─────────────────────────────────────────────────────
         * 状态: PRESSED (已按下)
         * 等待松开(短按) 或 长按超时
         * hold_cnt 每10ms+1，到80(=800ms)触发长按
         *─────────────────────────────────────────────────────*/
        case KEY_STATE_PRESSED:
            if (raw == 0) {                             /*仍在按住*/
                keys[i].hold_cnt++;
                if (keys[i].hold_cnt >= (KEY_LONG_PRESS_MS / KEY_SCAN_PERIOD_MS)) {
                    /*hold_cnt >= 80 (800ms/10ms) → 长按触发*/
                    keys[i].state = KEY_STATE_LONG;     /*进入长按状态*/
                    keys[i].hold_cnt = 0;
                    keys[i].long_flag = 1;              /*触发"长按"事件*/
                }
            } else {
                keys[i].state = KEY_STATE_IDLE;         /*松开了(短按结束)*/
                keys[i].hold_cnt = 0;
            }
            break;

        /*─────────────────────────────────────────────────────
         * 状态: LONG (长按已触发)
         * 等待松开，期间不产生任何新事件 (防止连续触发)
         *─────────────────────────────────────────────────────*/
        case KEY_STATE_LONG:
            if (raw == 1) {                             /*松开了*/
                keys[i].state = KEY_STATE_IDLE;         /*回到空闲*/
                keys[i].hold_cnt = 0;
            }
            /*raw==0: 继续保持LONG，不做任何事*/
            break;
        }
    }
}

/**
 * 查询按键是否"刚按下" (边沿触发, 读后即清)
 * press_flag 由 Key_Scan 在按下瞬间置1, 锁存到此函数读取
 * 读取后自动清零, 保证同一事件只被消费一次
 *
 * 使用场景: 菜单选择、确认进入等"单次触发"操作
 */
uint8_t Key_IsPressed(uint8_t key_id)
{
    uint8_t flag;
    if (key_id >= KEY_NUM) return 0;        /*越界保护*/
    flag = keys[key_id].press_flag;         /*读取标志*/
    keys[key_id].press_flag = 0;            /*读后清零 (锁存模式)*/
    return flag;
}

/**
 * 查询按键是否"长按触发" (边沿触发, 读后即清)
 * long_flag 由 Key_Scan 在长按800ms瞬间置1, 锁存到此函数读取
 * 读取后自动清零
 *
 * 使用场景: 秒表长按退出、闹钟长按开关等"次要功能"
 */
uint8_t Key_IsLongPress(uint8_t key_id)
{
    uint8_t flag;
    if (key_id >= KEY_NUM) return 0;        /*越界保护*/
    flag = keys[key_id].long_flag;          /*读取标志*/
    keys[key_id].long_flag = 0;             /*读后清零 (锁存模式)*/
    return flag;
}

/**
 * 查询按键是否"正在按住" (电平触发)
 * 每次调用都返回实时按下状态
 *
 * 使用场景: 需要持续检测按住状态的场景
 */
uint8_t Key_IsHeld(uint8_t key_id)
{
    if (key_id >= KEY_NUM) return 0;        /*越界保护*/
    return (Key_ReadPin(key_id) == 0);      /*直接读引脚: 0=按住*/
}
