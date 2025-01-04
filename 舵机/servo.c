#include "servo.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;

/* 舵机当前角度数组 */
static int16_t servo_angles[SERVO_MAX_NUM] = {0}; // 使用有符号整型以支持负角度

/* 定时器和通道配置数组 */
static const servo_config_t servo_config[SERVO_MAX_NUM] = {
    /* SERVO_ID_1: 180度舵机 */
    {&SERVO_1_TIMER, SERVO_1_CHANNEL, SERVO_1_MIN_ANGLE, SERVO_1_MAX_ANGLE},

    /* SERVO_ID_2: 180度舵机 */
    {&SERVO_2_TIMER, SERVO_2_CHANNEL, SERVO_2_MIN_ANGLE, SERVO_2_MAX_ANGLE},

    /* SERVO_ID_3: 270度舵机 */
    {&SERVO_3_TIMER, SERVO_3_CHANNEL, SERVO_3_MIN_ANGLE, SERVO_3_MAX_ANGLE},

    /* SERVO_ID_4: 270度舵机 */
    {&SERVO_4_TIMER, SERVO_4_CHANNEL, SERVO_4_MIN_ANGLE, SERVO_4_MAX_ANGLE},

    /* SERVO_ID_5: 180度舵机 */
    {&SERVO_5_TIMER, SERVO_5_CHANNEL, SERVO_5_MIN_ANGLE, SERVO_5_MAX_ANGLE}
};

/**
 * @brief  舵机初始化
 * @param  None
 * @retval None
 */
void servo_init(void)
{
    // 启动所有舵机的PWM输出
    for(uint8_t i = 0; i < SERVO_MAX_NUM; i++) {
        HAL_TIM_PWM_Start(servo_config[i].timer, servo_config[i].channel);
    }

    // 将所有舵机置于中心位置
    servo_reset_all();
}

/**
 * @brief  设置舵机角度
 * @param  servo_id: 舵机ID
 * @param  angle: 目标角度，相对于中心点的角度，可为负值
 * @retval None
 */
void servo_set_angle(servo_id_t servo_id, int16_t angle)
{
    if(servo_id >= SERVO_MAX_NUM) {
        return;
    }

    // 获取舵机配置
    const servo_config_t *config = &servo_config[servo_id];

    // 限制角度范围
    if(angle < config->servo_min_angle) {
        angle = config->servo_min_angle;
    }
    if(angle > config->servo_max_angle) {
        angle = config->servo_max_angle;
    }

    // 计算PWM脉宽
    uint32_t pulse = 1500 + ((angle * 1000) / 90); // 适用于-90到+90度

    // 设置PWM脉宽
    __HAL_TIM_SET_COMPARE(config->timer, config->channel, pulse);

    // 更新当前角度
    servo_angles[servo_id] = angle;
}

/**
 * @brief  获取舵机当前角度
 * @param  servo_id: 舵机ID
 * @retval 当前角度值，相对于中心点的角度
 */
int16_t servo_get_angle(servo_id_t servo_id)
{
    if(servo_id >= SERVO_MAX_NUM) {
        return 0;
    }
    return servo_angles[servo_id];
}

/**
 * @brief  复位所有舵机到中心位置
 * @param  None
 * @retval None
 */
void servo_reset_all(void)
{
    for(uint8_t i = 0; i < SERVO_MAX_NUM; i++) {
        servo_set_angle((servo_id_t)i, 0);
    }
}

/**
 * @brief  设置机械臂位置（所有舵机）
 * @param  angles: 角度数组指针，包含5个舵机的目标角度
 * @retval None
 */
void servo_set_arm_position(int16_t *angles)
{
    for(uint8_t i = 0; i < SERVO_MAX_NUM; i++) {
        servo_set_angle((servo_id_t)i, angles[i]);
    }
}
