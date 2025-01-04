#include "led.h"

/**
 * @brief LED1��
 * @note ����LED1����Ϊ�ߵ�ƽ
 */
void LED_01_ON(void)
{
    HAL_GPIO_WritePin(LED_01_GPIO_PORT, LED_01_GPIO_PIN, GPIO_PIN_SET);
}

/**
 * @brief LED1�ر�
 * @note ����LED1����Ϊ�͵�ƽ
 */
void LED_01_OFF(void)
{
    HAL_GPIO_WritePin(LED_01_GPIO_PORT, LED_01_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief LED1��ת
 * @note ��תLED1���ŵ�ƽ״̬
 */
void LED_01_TOGGLE(void)
{
    HAL_GPIO_TogglePin(LED_01_GPIO_PORT, LED_01_GPIO_PIN);
}

/**
 * @brief LED2��
 * @note ����LED2����Ϊ�ߵ�ƽ
 */
void LED_02_ON(void)
{
    HAL_GPIO_WritePin(LED_02_GPIO_PORT, LED_02_GPIO_PIN, GPIO_PIN_SET);
}

/**
 * @brief LED2�ر�
 * @note ����LED2����Ϊ�͵�ƽ
 */
void LED_02_OFF(void)
{
    HAL_GPIO_WritePin(LED_02_GPIO_PORT, LED_02_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief LED2��ת
 * @note ��תLED2���ŵ�ƽ״̬
 */
void LED_02_TOGGLE(void)
{
    HAL_GPIO_TogglePin(LED_02_GPIO_PORT, LED_02_GPIO_PIN);
}

/**
 * @brief LED3��
 * @note ����LED3����Ϊ�ߵ�ƽ
 */
void LED_03_ON(void)
{
    HAL_GPIO_WritePin(LED_03_GPIO_PORT, LED_03_GPIO_PIN, GPIO_PIN_SET);
}

/**
 * @brief LED3�ر�
 * @note ����LED3����Ϊ�͵�ƽ
 */
void LED_03_OFF(void)
{
    HAL_GPIO_WritePin(LED_03_GPIO_PORT, LED_03_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief LED3��ת
 * @note ��תLED3���ŵ�ƽ״̬
 */
void LED_03_TOGGLE(void)
{
    HAL_GPIO_TogglePin(LED_03_GPIO_PORT, LED_03_GPIO_PIN);
}
