#ifndef _DEV_UART_H_
#define _DEV_UART_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "bsp_uart.h"
#include "fifo.h"

/************************************* 以下为参数宏定义 *******************************************/
/******************* 移植需要修改的宏定义 ******************/
/* 串口缓存大小 */
#define UART1_TX_BUF_SIZE           1024
#define UART1_RX_BUF_SIZE           1024
#define	UART1_DMA_RX_BUF_SIZE       128
#define	UART1_DMA_TX_BUF_SIZE       128

#define UART3_TX_BUF_SIZE           1024
#define UART3_RX_BUF_SIZE           1024
#define	UART3_DMA_RX_BUF_SIZE       128
#define	UART3_DMA_TX_BUF_SIZE       128

#define UART4_TX_BUF_SIZE           1024
#define UART4_RX_BUF_SIZE           1024
#define	UART4_DMA_RX_BUF_SIZE       128
#define	UART4_DMA_TX_BUF_SIZE       128

#define UART5_TX_BUF_SIZE           1024
#define UART5_RX_BUF_SIZE           1024
#define	UART5_DMA_RX_BUF_SIZE       128
#define	UART5_DMA_TX_BUF_SIZE       128
/******************* 移植需要修改的宏定义 ******************/

/************************************** 以下为结构体声明 ******************************************/
/* 串口设备数据结构 */
typedef struct
{
    uint8_t status;                 /* 发送状态 */
    _fifo_t tx_fifo;                /* 发送fifo */
    _fifo_t rx_fifo;                /* 接收fifo */
    uint8_t *dmarx_buf;	            /* dma接收缓存 */
    uint16_t dmarx_buf_size;        /* dma接收缓存大小*/
    uint8_t *dmatx_buf;	            /* dma发送缓存 */
    uint16_t dmatx_buf_size;        /* dma发送缓存大小 */
    uint16_t last_dmarx_size;       /* dma上一次接收数据大小 */
} uart_device_t;

/* 清空缓冲区的返回状态 */
typedef enum {
    UART_FLUSH_OK = 0,              /* 清空成功 */
    UART_FLUSH_TIMEOUT,             /* 清空超时 */
    UART_FLUSH_ERROR                /* 清空错误 */
} uart_flush_status_t;

/* 清空缓冲区的配置参数 */
typedef struct {
    uint32_t timeout_ms;            /* 超时时间(毫秒) */
    uint16_t chunk_size;            /* 每次读取大小 */
    uint8_t delay_ms;               /* 每次读取间隔(毫秒) */
} uart_flush_config_t;

/************************************** 以下为函数声明 ******************************************/
void uart_device_init(uint8_t uart_id);                                                     /* 串口设备初始化 */
uint16_t uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t size);                    /* 串口写数据 */
uint16_t uart_read(uint8_t uart_id, uint8_t *buf, uint16_t size);                           /* 串口读数据 */
void uart_dmarx_done_isr(uint8_t uart_id);                                                  /* DMA接收完成中断处理 */
void uart_dmarx_half_done_isr(uint8_t uart_id);                                             /* DMA接收半完成中断处理 */
void uart_dmarx_idle_isr(uint8_t uart_id);                                                  /* 串口空闲中断处理 */
void uart_dmatx_done_isr(uint8_t uart_id);                                                  /* DMA发送完成中断处理 */
void uart_poll_dma_tx(uint8_t uart_id);                                                     /* 轮询DMA发送 */
uint8_t uart_set_baudrate(uint8_t uart_id, uint32_t baudrate);                              /* 设置串口波特率 */
uart_flush_status_t uart_flush_buffer(uint8_t uart_id, const uart_flush_config_t *config);  /* 清空串口缓冲区,带配置参数 */
uart_flush_status_t uart_flush(uint8_t uart_id);                                            /* 清空串口缓冲区,使用默认配置 */
int uart1_print(const char *str, uint16_t len);                                             /* 串口1直接打印字符串 */
int uart1_printf(const char *format, ...);                                                  /* 串口1格式化打印 */
int uart3_printf(const char *format, ...);                                                  /* 串口3格式化打印 */
int uart4_printf(const char *format, ...);                                                  /* 串口4格式化打印 */
int uart5_printf(const char *format, ...);                                                  /* 串口5格式化打印 */

#endif
