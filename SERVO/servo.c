/************************************** 舵机驱动使用说明 ******************************************/
/* 
 * 1. 舵机角度控制说明：
 *    - 所有舵机角度以中心点(0度)为基准，支持正负角度
 *    - SERVO_ID_1/2/5 (180度舵机): 角度范围 -90 到 +90
 *    - SERVO_ID_3/4 (270度舵机): 角度范围 -135 到 +135
 *
 * 2. 直接控制方式（不平滑）：
 *    - 单个舵机控制:
 *      servo_set_angle(SERVO_ID_1, 45);  // 设置舵机1到45度
 *    
 *    - 多个舵机同时控制:
 *      int16_t angles[5] = {45, -45, 90, -90, 0};  // 设置5个舵机的目标角度
 *      servo_set_arm_position(angles);
 *
 * 3. 平滑控制方式：
 *    - 单个舵机平滑控制:
 *      // 参数: 舵机ID, 目标角度, 步进延时(ms), 步进角度
 *      servo_set_single_position_smooth(SERVO_ID_1, 45, 20, 2);  // 每20ms移动2度
 *    
 *    - 多个舵机同时平滑控制:
 *      int16_t angles[5] = {45, -45, 90, -90, 0};
 *      servo_set_arm_position_smooth(angles, 20, 2);  // 所有舵机每20ms移动2度
 *    
 *    注意：使用平滑控制时，需要在主循环或定时器中断中调用 servo_smooth_update()
 *
 * 4. 其他功能：
 *    - 获取当前角度: servo_get_angle(servo_id)
 *    - 复位所有舵机: servo_reset_all()  // 所有舵机回到0度位置
 */

#include "servo.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;

/* 舵机当前角度数组 */
static int16_t servo_angles[SERVO_MAX_NUM] = {0}; // 使用有符号整型以支持负角度

/* 舵机移动结构体数组 */
static volatile servo_move_s servo_moves[SERVO_MAX_NUM] = {0};

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

/**
 * @brief 平滑设置单个舵机的目标角度
 * @param servo_id: 舵机ID
 * @param target_angle: 目标角度，相对于中心点的角度
 * @param step_delay_ms: 每步延时（毫秒）
 * @param step_angle: 每步角度变化
 */
void servo_set_single_position_smooth(servo_id_t servo_id, int16_t target_angle, uint16_t step_delay_ms, int16_t step_angle)
{
    if(servo_id >= SERVO_MAX_NUM) {
        return;
    }

    servo_moves[servo_id].target_angle = target_angle;
    servo_moves[servo_id].step_angle = step_angle;
    servo_moves[servo_id].step_delay_ms = step_delay_ms;
    servo_moves[servo_id].current_delay = 0;
    servo_moves[servo_id].is_moving = true;
}

/**
 * @brief 平滑设置机械臂所有舵机的目标角度
 * @param angles: 角度数组指针，包含5个舵机的目标角度
 * @param step_delay_ms: 每步延时（毫秒）
 * @param step_angle: 每步角度变化
 */
void servo_set_arm_position_smooth(int16_t *angles, uint16_t step_delay_ms, int16_t step_angle)
{
    for(uint8_t i = 0; i < SERVO_MAX_NUM; i++) {
        servo_set_single_position_smooth((servo_id_t)i, angles[i], step_delay_ms, step_angle);
    }
}

/**
 * @brief 平滑更新所有舵机的位置，需要定时调用
 */
void servo_smooth_update(void)
{
    for(uint8_t i = 0; i < SERVO_MAX_NUM; i++) {
        if(servo_moves[i].is_moving) {
            servo_moves[i].current_delay += 10; // 假设此函数每10ms调用一次

            if(servo_moves[i].current_delay >= servo_moves[i].step_delay_ms) {
                servo_moves[i].current_delay = 0;

                // 获取当前角度
                int16_t current_angle = servo_angles[i];
                int16_t target_angle = servo_moves[i].target_angle;
                int16_t step_angle = servo_moves[i].step_angle;

                // 计算新的角度
                if(current_angle < target_angle) {
                    current_angle += step_angle;
                    if(current_angle > target_angle) {
                        current_angle = target_angle;
                    }
                }
                else if(current_angle > target_angle) {
                    current_angle -= step_angle;
                    if(current_angle < target_angle) {
                        current_angle = target_angle;
                    }
                }

                // 设置新的角度
                servo_set_angle((servo_id_t)i, current_angle);

                // 检查是否到达目标
                if(current_angle == target_angle) {
                    servo_moves[i].is_moving = false;
                }
            }
        }
    }
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
    int16_t target_angles[SERVO_MAX_NUM] = {0, 0, 0, 0, 0};
    servo_set_arm_position(target_angles);
}
