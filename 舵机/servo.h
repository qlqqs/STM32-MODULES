#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

/******************************************************************************************/
/******************* 移植需要修改的宏定义 ******************/
/* 舵机定义 */
#define SERVO_1_TIMER                  htim2                /* 舵机1定时器句柄 */
#define SERVO_1_CHANNEL                TIM_CHANNEL_2        /* 舵机1定时器通道 */
#define SERVO_1_MIN_ANGLE              -90                  /* 舵机1最小角度（相对于中心点） */
#define SERVO_1_MAX_ANGLE              90                   /* 舵机1最大角度（相对于中心点） */

#define SERVO_2_TIMER                  htim2                /* 舵机2定时器句柄 */
#define SERVO_2_CHANNEL                TIM_CHANNEL_1        /* 舵机2定时器通道 */
#define SERVO_2_MIN_ANGLE              -90                  /* 舵机2最小角度（相对于中心点） */
#define SERVO_2_MAX_ANGLE              90                   /* 舵机2最大角度（相对于中心点） */

#define SERVO_3_TIMER                  htim3                /* 舵机3定时器句柄 */
#define SERVO_3_CHANNEL                TIM_CHANNEL_1        /* 舵机3定时器通道 */
#define SERVO_3_MIN_ANGLE              -135                 /* 舵机3最小角度（相对于中心点） */
#define SERVO_3_MAX_ANGLE              135                  /* 舵机3最大角度（相对于中心点） */

#define SERVO_4_TIMER                  htim3                /* 舵机4定时器句柄 */
#define SERVO_4_CHANNEL                TIM_CHANNEL_2        /* 舵机4定时器通道 */
#define SERVO_4_MIN_ANGLE              -135                 /* 舵机4最小角度（相对于中心点） */
#define SERVO_4_MAX_ANGLE              135                  /* 舵机4最大角度（相对于中心点） */
    
#define SERVO_5_TIMER                  htim8                /* 舵机5定时器句柄 */
#define SERVO_5_CHANNEL                TIM_CHANNEL_1        /* 舵机5定时器通道 */
#define SERVO_5_MIN_ANGLE              -90                  /* 舵机5最小角度（相对于中心点） */
#define SERVO_5_MAX_ANGLE              90                   /* 舵机5最大角度（相对于中心点） */
/******************* 移植需要修改的宏定义 ******************/

/************************************** 以下为结构体声明 ******************************************/
/* 舵机ID结构体定义 */
typedef enum {
    SERVO_ID_1 = 0,    /* 底层舵机（180度） */
    SERVO_ID_2,        /* 第二层舵机（180度） */
    SERVO_ID_3,        /* 第三层舵机（270度） */
    SERVO_ID_4,        /* 第四层舵机（270度） */
    SERVO_ID_5,        /* 夹爪舵机（180度） */
    SERVO_MAX_NUM
} servo_id_t;

/* 舵机配置结构体定义 */
typedef struct {
    TIM_HandleTypeDef* timer;      /* 定时器句柄 */
    uint32_t channel;              /* 定时器通道 */
    int16_t servo_min_angle;       /* 舵机最小角度（相对于中心点） */
    int16_t servo_max_angle;       /* 舵机最大角度（相对于中心点） */
} servo_config_t;

/************************************** 以下为函数声明 ******************************************/
/* 函数声明 */
void servo_init(void);                                      /* 舵机初始化 */
void servo_set_angle(servo_id_t servo_id, int16_t angle);   /* 设置舵机角度，支持负角度 */
int16_t servo_get_angle(servo_id_t servo_id);               /* 获取舵机当前角度 */
void servo_reset_all(void);                                 /* 复位所有舵机到中心位置 */
void servo_set_arm_position(int16_t *angles);              /* 设置机械臂位置（所有舵机） */

#endif /* __SERVO_H */
