#ifndef __KEY_H
#define __KEY_H

#include "main.h"

/************************************* 以下为参数宏定义 *******************************************/
/******************* 移植需要修改的宏定义 ******************/
/* 按键引脚定义 */
#define KEY_01_GPIO_PIN                   KEY_01_Pin
#define KEY_01_GPIO_PORT                  KEY_01_GPIO_Port
#define KEY_02_GPIO_PIN                   KEY_02_Pin
#define KEY_02_GPIO_PORT                  KEY_02_GPIO_Port
/******************* 移植需要修改的宏定义 ******************/

/* 按键时间参数定义 */
#define KEY_DEBOUNCE_TIME    20      // 消抖时间，单位ms
#define KEY_SHORT_TIME       30      // 短按最短时间，单位ms
#define KEY_LONG_TIME        1000    // 长按时间阈值，单位ms
#define KEY_DOUBLE_TIME      300     // 双击间隔时间，单位ms
#define KEY_LONG_REPEAT_TIME 500     // 长按重复触发间隔，单位ms

/* 按键状态定义 */
#define KEY_RELEASED    1    // 按键释放
#define KEY_PRESSED     0    // 按键按下

/************************************** 以下为结构体声明 ******************************************/
/* 按键ID定义 */
typedef enum {
    KEY_01_ID = 0,
    KEY_02_ID,
    KEY_MAX_NUM
} key_id_t;

/* 按键事件类型 */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT_PRESS,    // 短按
    KEY_EVENT_LONG_PRESS,     // 长按
    KEY_EVENT_DOUBLE_CLICK,   // 双击
    KEY_EVENT_RELEASE         // 释放
} key_event_t;

/* 按键信息结构体 */
typedef struct {
    uint8_t state;                   // 当前状态
    uint8_t last_state;              // 上一次状态
    uint32_t press_time;             // 按下时间点
    uint32_t release_time;           // 释放时间点
    uint32_t last_press;             // 上次按下时间点(用于双击检测)
    uint8_t press_count;             // 按下次数(用于双击检测)
    key_event_t event;               // 按键事件
    uint32_t last_long_press_time;   // 上次长按触发时间
} key_info_t;

/************************************** 以下为函数声明 ******************************************/
/* 外部函数声明 */
void key_init(void);
void key_exti_callback(uint16_t GPIO_Pin);
void key_update(void);
key_event_t key_get_event(key_id_t key_id);

#endif
