#ifndef __LED_H
#define __LED_H

/* 手电筒 (PB0 推挽输出, 低电平点亮) */
void Flashlight_Init(void);
void Flashlight_ON(void);
void Flashlight_OFF(void);
void Flashlight_Turn(void);

#endif
