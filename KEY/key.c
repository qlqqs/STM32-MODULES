/**
 * @brief ��������ģ��
 * @details ֧�����¹���:
 *          1. ֧�ֶ������ͬʱ���
 *          2. ֧�ְ�������
 *          3. ֧�ֶ̰���������˫�����
 *          4. ֧�ֳ����ظ�����
 *          5. ֧�ְ����ͷż��
 *          6. ʹ���жϷ�ʽ��ⰴ��״̬,����CPUռ��
 * @note    ��ֲʱ��Ҫ:
 *          1. �����������Ŷ���(key.h�е�KEY_xx_GPIO_PIN��KEY_xx_GPIO_PORT)
 *          2. ���жϷ�������е���key_exti_callback()
 *          3. ����ѭ����ʱ���ж��е���key_update()
 * @author  qlqqs
 * @date    2024.12.19
 */

#include "key.h"

/* ������Ϣ���� */
static key_info_t key_info[KEY_MAX_NUM] = {0};

/* GPIO�������� */
static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} key_gpio[KEY_MAX_NUM] = {
    {KEY_01_GPIO_PORT, KEY_01_GPIO_PIN},
    {KEY_02_GPIO_PORT, KEY_02_GPIO_PIN}
};

/**
 * @brief ������ʼ��
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
 * @brief �����ⲿ�жϻص�����
 */
void key_exti_callback(uint16_t GPIO_Pin)
{
    uint32_t current_tick = HAL_GetTick();

    for(uint8_t i = 0; i < KEY_MAX_NUM; i++) {
        if(GPIO_Pin == key_gpio[i].pin) {
            key_info[i].last_state = key_info[i].state;
            key_info[i].state = HAL_GPIO_ReadPin(key_gpio[i].port, key_gpio[i].pin);

            if(key_info[i].state == KEY_PRESSED) {   // ��������
                key_info[i].press_time = current_tick;
            } else {                                 // �����ͷ�
                key_info[i].release_time = current_tick;
            }
            break;
        }
    }
}

/**
 * @brief ����״̬���º��¼����
 */
void key_update(void)
{
    uint32_t current_tick = HAL_GetTick();

    for(uint8_t i = 0; i < KEY_MAX_NUM; i++) {
        // 1. ��������״̬���� (��鳤��)
        if(key_info[i].state == KEY_PRESSED) {
            // �״μ�⵽����
            if(current_tick - key_info[i].press_time >= KEY_LONG_TIME && key_info[i].event == KEY_EVENT_NONE) {
                key_info[i].event = KEY_EVENT_LONG_PRESS;
                key_info[i].last_long_press_time = current_tick;
                // �������� press_time �Ա��´��ظ�����
                key_info[i].press_time = current_tick;
            }
            // ���������������Դ��������¼�
            else if(current_tick - key_info[i].press_time >= KEY_LONG_REPEAT_TIME) {
                key_info[i].event = KEY_EVENT_LONG_PRESS;
                key_info[i].last_long_press_time = current_tick;
                // �������� press_time �Ա��´��ظ�����
                key_info[i].press_time = current_tick;
            }
        }
        // 2. �����ͷ�״̬����
        else if(key_info[i].state == KEY_RELEASED && key_info[i].last_state == KEY_PRESSED) {
            uint32_t press_duration = key_info[i].release_time - key_info[i].press_time;

            // �����ж�
            if(press_duration >= KEY_DEBOUNCE_TIME) {
                // ˫�����
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

                // �̰���� (����δ��⵽������˫��)
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
 * @brief ��ȡ�����¼������
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
