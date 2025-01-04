#ifndef _LED_H
#define _LED_H

#include "main.h"

/************************************** 以下为参数宏定义 ******************************************/
/******************* 移植需要修改的宏定义 ******************/
/* 引脚 定义 */
/* LED1引脚定义 */
#define LED_01_GPIO_PIN                 LED_01_Pin        /* LED1引脚号 */
#define LED_01_GPIO_PORT                LED_01_GPIO_Port  /* LED1端口 */

/* LED2引脚定义 */
#define LED_02_GPIO_PIN                 LED_02_Pin        /* LED2引脚号 */
#define LED_02_GPIO_PORT                LED_02_GPIO_Port  /* LED2端口 */

/* LED3引脚定义 */
#define LED_03_GPIO_PIN                 LED_03_Pin        /* LED3引脚号 */
#define LED_03_GPIO_PORT                LED_03_GPIO_Port  /* LED3端口 */
/******************* 移植需要修改的宏定义 ******************/

/************************************** 以下为函数声明 ******************************************/
/* 外部接口函数*/
void LED_01_ON(void);                                     /* LED1打开 */
void LED_01_OFF(void);                                    /* LED1关闭 */
void LED_01_TOGGLE(void);                                 /* LED1翻转 */

void LED_02_ON(void);                                     /* LED2打开 */
void LED_02_OFF(void);                                    /* LED2关闭 */
void LED_02_TOGGLE(void);                                 /* LED2翻转 */

void LED_03_ON(void);                                     /* LED3打开 */
void LED_03_OFF(void);                                    /* LED3关闭 */
void LED_03_TOGGLE(void);                                 /* LED3翻转 */

#endif
