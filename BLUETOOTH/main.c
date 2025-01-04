/* USER CODE BEGIN Includes */

/* 此模块依赖于串口模块 */
#include "dev_uart.h"
#include "atk_ble03.h"

/* USER CODE END Includes */


    /* USER CODE BEGIN 2 */
    uart_device_init(DEV_UART1);            /* 初始化 串口1 */
    uart_device_init(DEV_UART3);            /* 初始化 串口3 */
    uart_device_init(DEV_UART4);            /* 初始化 串口4 */
    uart_device_init(DEV_UART5);            /* 初始化 串口5 */

    atk_ble03_init();                       /* ATK-BLE03蓝牙初始化 */


    /* USER CODE END 2 */

    while (1)
    {
        /* 测试蓝牙透传 */
        uint8_t buf[128];
        uint16_t len;
        
        /* 从串口1读取数据,转发到串口5(蓝牙) */
        len = uart_read(DEV_UART1, buf, sizeof(buf));
        if(len > 0) {
            uart_write(DEV_UART5, buf, len);
            uart_poll_dma_tx(DEV_UART5);
        }
        
        /* 从串口5(蓝牙)读取数据,转发到串口1 */
        len = uart_read(DEV_UART5, buf, sizeof(buf));
        if(len > 0) {
            uart_write(DEV_UART1, buf, len);
            uart_poll_dma_tx(DEV_UART1);
        }
    }