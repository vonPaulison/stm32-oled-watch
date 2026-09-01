#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "watch.h"
#include "MyRTC.h"
#include "Timer.h"             /*TIM2 1ms中断 — 秒表计时基准*/
#include "LED.h"               /*LED+手电筒 GPIO 初始化*/
/**
  * 坐标轴定义：
  * 左上角为(0, 0)点
  * 横向向右为X轴，取值范围：0~127
  * 纵向向下为Y轴，取值范围：0~63
  * 
  *       0             X轴           127 
  *      .------------------------------->
  *    0 |
  *      |
  *      |
  *      |
  *  Y轴 |
  *      |
  *      |
  *      |
  *   63 |
  *      v
  * 
  */

int main(void)
{
    OLED_Init();
    Watch_Init();              /*初始化按键+菜单*/
    MyRTC_Init();
    Timer_Init();              /*启动TIM2 1ms中断 (秒表计时基准)*/
    Flashlight_Init();         /*初始化手电筒 GPIO (PB0)*/
    while (1)
    {
        Watch_Tick();          /*主循环 (秒表精度已不受主循环速度影响)*/
        Delay_ms(10);
    }
}
