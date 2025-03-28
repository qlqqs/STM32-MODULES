
#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include <stdint.h>
#include <stdbool.h>

/************************************** 以下为参数宏定义 ******************************************/
/******************* 移植需要修改的宏定义 ******************/
/* 串口设备ID定义 */
#define DEV_UART1                1
#define DEV_UART3                3
#define DEV_UART4                4
#define DEV_UART5                5

/* 串口设备数量定义 */
#define UART_DEVICE_MAX_NUM      5                                 /* 最大串口设备数量 这里最大为UART5*/
#define UART_DEVICE_ARRAY_NUM    (UART_DEVICE_MAX_NUM + 1)         /* 串口设备数组大小 UART_DEVICE_MAX_NUM + 1*/
#define UART_COUNT_ARRAY_SIZE    ((UART_DEVICE_MAX_NUM + 1) * 2)   /* 串口收发计数数组大小 2*UART_DEVICE_MAX_NUM */
/******************* 移植需要修改的宏定义 ******************/

/* 串口设置波特率状态定义 */
#define UART_SET_BAUDRATE_OK     0                                 /* 设置波特率成功 */
#define UART_SET_BAUDRATE_ERROR  1                                 /* 设置波特率失败 */

/************************************** 以下为函数声明 ******************************************/
/* 外部函数声明 */
void bsp_uart_dmatx_config(uint8_t uart_id, uint8_t *mem_addr, uint32_t mem_size);
void bsp_uart_dmarx_config(uint8_t uart_id, uint8_t *mem_addr, uint32_t mem_size);
void uart_dma_init(uint8_t uart_id, uint8_t *mem_addr, uint32_t mem_size);
uint16_t bsp_uart_get_dmarx_buf_remain_size(uint8_t uart_id);

#endif /* _BSP_UART_H_ */
