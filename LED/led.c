#include "led.h"

/**
 * @brief LED1打开
 * @note 设置LED1引脚为高电平
 */
void LED_01_ON(void)
{
    HAL_GPIO_WritePin(LED_01_GPIO_PORT, LED_01_GPIO_PIN, GPIO_PIN_SET);
}

/**
 * @brief LED1关闭
 * @note 设置LED1引脚为低电平
 */
void LED_01_OFF(void)
{
    HAL_GPIO_WritePin(LED_01_GPIO_PORT, LED_01_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief LED1翻转
 * @note 翻转LED1引脚电平状态
 */
void LED_01_TOGGLE(void)
{
    HAL_GPIO_TogglePin(LED_01_GPIO_PORT, LED_01_GPIO_PIN);
}

/**
 * @brief LED2打开
 * @note 设置LED2引脚为高电平
 */
void LED_02_ON(void)
{
    HAL_GPIO_WritePin(LED_02_GPIO_PORT, LED_02_GPIO_PIN, GPIO_PIN_SET);
}

/**
 * @brief LED2关闭
 * @note 设置LED2引脚为低电平
 */
void LED_02_OFF(void)
{
    HAL_GPIO_WritePin(LED_02_GPIO_PORT, LED_02_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief LED2翻转
 * @note 翻转LED2引脚电平状态
 */
void LED_02_TOGGLE(void)
{
    HAL_GPIO_TogglePin(LED_02_GPIO_PORT, LED_02_GPIO_PIN);
}

/**
 * @brief LED3打开
 * @note 设置LED3引脚为高电平
 */
void LED_03_ON(void)
{
    HAL_GPIO_WritePin(LED_03_GPIO_PORT, LED_03_GPIO_PIN, GPIO_PIN_SET);
}

/**
 * @brief LED3关闭
 * @note 设置LED3引脚为低电平
 */
void LED_03_OFF(void)
{
    HAL_GPIO_WritePin(LED_03_GPIO_PORT, LED_03_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief LED3翻转
 * @note 翻转LED3引脚电平状态
 */
void LED_03_TOGGLE(void)
{
    HAL_GPIO_TogglePin(LED_03_GPIO_PORT, LED_03_GPIO_PIN);
}
