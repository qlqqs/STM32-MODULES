/**
 * @brief 按键驱动模块
 * @details 支持以下功能:
 *          1. 支持多个按键同时检测
 *          2. 支持按键消抖
 *          3. 支持短按、长按、双击检测
 *          4. 支持长按重复触发
 *          5. 支持按键释放检测
 *          6. 使用中断方式检测按键状态,降低CPU占用
 * @note    移植时需要:
 *          1. 增减按键引脚定义(key.h中的KEY_xx_GPIO_PIN和KEY_xx_GPIO_PORT)
 *          2. 在中断服务程序中调用key_exti_callback()
 *          3. 在主循环或定时器中断中调用key_update()
 * @author  qlqqs
 * @date    2024.12.19
 */

#include "key.h"

/* 按键信息数组 */
static key_info_t key_info[KEY_MAX_NUM] = {0};

/* GPIO配置数组 */
static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} key_gpio[KEY_MAX_NUM] = {
    {KEY_01_GPIO_PORT, KEY_01_GPIO_PIN},
    {KEY_02_GPIO_PORT, KEY_02_GPIO_PIN}
};

/**
 * @brief 按键初始化
 */
void key_init(void)
{
    for(uint8_t i = 0; i < KEY_MAX_NUM; i++) {
        key_info[i].state = KEY_RELEASED;
        key_info[i].last_state = KEY_RELEASED;
        key_info[i].press_time = 0;
        key_info[i].release_time = 0;
        key_info[i].last_press = 0;
        key_info[i].press_count = 0;
        key_info[i].event = KEY_EVENT_NONE;
        key_info[i].last_long_press_time = 0;
    }
}

/**
 * @brief 按键外部中断回调函数
 */
void key_exti_callback(uint16_t GPIO_Pin)
{
    uint32_t current_tick = HAL_GetTick();

    for(uint8_t i = 0; i < KEY_MAX_NUM; i++) {
        if(GPIO_Pin == key_gpio[i].pin) {
            key_info[i].last_state = key_info[i].state;
            key_info[i].state = HAL_GPIO_ReadPin(key_gpio[i].port, key_gpio[i].pin);

            if(key_info[i].state == KEY_PRESSED) {   // 按键按下
                key_info[i].press_time = current_tick;
            } else {                                 // 按键释放
                key_info[i].release_time = current_tick;
            }
            break;
        }
    }
}

/**
 * @brief 按键状态更新和事件检测
 */
void key_update(void)
{
    uint32_t current_tick = HAL_GetTick();

    for(uint8_t i = 0; i < KEY_MAX_NUM; i++) {
        // 1. 按键按下状态处理 (检查长按)
        if(key_info[i].state == KEY_PRESSED) {
            // 首次检测到长按
            if(current_tick - key_info[i].press_time >= KEY_LONG_TIME && key_info[i].event == KEY_EVENT_NONE) {
                key_info[i].event = KEY_EVENT_LONG_PRESS;
                key_info[i].last_long_press_time = current_tick;
                // 重新设置 press_time 以便下次重复触发
                key_info[i].press_time = current_tick;
            }
            // 持续长按，周期性触发长按事件
            else if(current_tick - key_info[i].press_time >= KEY_LONG_REPEAT_TIME) {
                key_info[i].event = KEY_EVENT_LONG_PRESS;
                key_info[i].last_long_press_time = current_tick;
                // 重新设置 press_time 以便下次重复触发
                key_info[i].press_time = current_tick;
            }
        }
        // 2. 按键释放状态处理
        else if(key_info[i].state == KEY_RELEASED && key_info[i].last_state == KEY_PRESSED) {
            uint32_t press_duration = key_info[i].release_time - key_info[i].press_time;

            // 消抖判断
            if(press_duration >= KEY_DEBOUNCE_TIME) {
                // 双击检测
                if(current_tick - key_info[i].last_press <= KEY_DOUBLE_TIME) {
                    key_info[i].press_count++;
                    if(key_info[i].press_count >= 2) {
                        key_info[i].event = KEY_EVENT_DOUBLE_CLICK;
                        key_info[i].press_count = 0;
                    }
                } else {
                    key_info[i].press_count = 1;
                }

                key_info[i].last_press = key_info[i].release_time;

                // 短按检测 (仅当未检测到长按和双击)
                if(press_duration >= KEY_SHORT_TIME &&
                        press_duration < KEY_LONG_TIME &&
                        key_info[i].event == KEY_EVENT_NONE) {
                    key_info[i].event = KEY_EVENT_SHORT_PRESS;
                }
            }

            key_info[i].last_state = key_info[i].state;
        }
    }
}

/**
 * @brief 获取按键事件并清除
 */
key_event_t key_get_event(key_id_t key_id)
{
    if(key_id >= KEY_MAX_NUM) {
        return KEY_EVENT_NONE;
    }

    key_event_t event = key_info[key_id].event;
    key_info[key_id].event = KEY_EVENT_NONE;
    return event;
}
