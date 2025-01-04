/* USER CODE BEGIN Includes */
#include "key.h"
/* USER CODE END Includes */

/**
  * @brief  GPIO外部中断回调函数
  * @param  GPIO_Pin: 触发中断的GPIO引脚
  * @retval 无
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    key_exti_callback(GPIO_Pin);
}

/* USER CODE END 1 */