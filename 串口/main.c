/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "dev_uart.h"

/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
    uart_device_init(DEV_UART1);        /* 开启串口1 */
    uart_device_init(DEV_UART3);        /* 开启串口3 */
    uart_device_init(DEV_UART4);        /* 开启串口4 */
    uart_device_init(DEV_UART5);        /* 开启串口5 */
/* USER CODE END 2 */


    static uint32_t s_count = 0;
    uint16_t size = 0;
    uint8_t buf[256] = {0};
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        s_count++;
	  	
        if (s_count%1000==0)
	  	{
            /* 串口数据回环测试 */
            size = uart_read(DEV_UART1, buf, 256);
            uart_write(DEV_UART1, buf, size);
            
            /* 将fifo数据拷贝到dma buf，并启动dma传输 */
            uart_poll_dma_tx(DEV_UART1); 
        }