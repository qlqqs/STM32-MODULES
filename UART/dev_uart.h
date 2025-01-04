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

/************************************* ����Ϊ�����궨�� *******************************************/
/******************* ��ֲ��Ҫ�޸ĵĺ궨�� ******************/
/* ���ڻ����С */
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
/******************* ��ֲ��Ҫ�޸ĵĺ궨�� ******************/

/************************************** ����Ϊ�ṹ������ ******************************************/
/* �����豸���ݽṹ */
typedef struct
{
    uint8_t status;                 /* ����״̬ */
    _fifo_t tx_fifo;                /* ����fifo */
    _fifo_t rx_fifo;                /* ����fifo */
    uint8_t *dmarx_buf;	            /* dma���ջ��� */
    uint16_t dmarx_buf_size;        /* dma���ջ����С*/
    uint8_t *dmatx_buf;	            /* dma���ͻ��� */
    uint16_t dmatx_buf_size;        /* dma���ͻ����С */
    uint16_t last_dmarx_size;       /* dma��һ�ν������ݴ�С */
} uart_device_t;

/* ��ջ������ķ���״̬ */
typedef enum {
    UART_FLUSH_OK = 0,              /* ��ճɹ� */
    UART_FLUSH_TIMEOUT,             /* ��ճ�ʱ */
    UART_FLUSH_ERROR                /* ��մ��� */
} uart_flush_status_t;

/* ��ջ����������ò��� */
typedef struct {
    uint32_t timeout_ms;            /* ��ʱʱ��(����) */
    uint16_t chunk_size;            /* ÿ�ζ�ȡ��С */
    uint8_t delay_ms;               /* ÿ�ζ�ȡ���(����) */
} uart_flush_config_t;

/************************************** ����Ϊ�������� ******************************************/
void uart_device_init(uint8_t uart_id);                                                     /* �����豸��ʼ�� */
uint16_t uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t size);                    /* ����д���� */
uint16_t uart_read(uint8_t uart_id, uint8_t *buf, uint16_t size);                           /* ���ڶ����� */
void uart_dmarx_done_isr(uint8_t uart_id);                                                  /* DMA��������жϴ��� */
void uart_dmarx_half_done_isr(uint8_t uart_id);                                             /* DMA���հ�����жϴ��� */
void uart_dmarx_idle_isr(uint8_t uart_id);                                                  /* ���ڿ����жϴ��� */
void uart_dmatx_done_isr(uint8_t uart_id);                                                  /* DMA��������жϴ��� */
void uart_poll_dma_tx(uint8_t uart_id);                                                     /* ��ѯDMA���� */
uint8_t uart_set_baudrate(uint8_t uart_id, uint32_t baudrate);                              /* ���ô��ڲ����� */
uart_flush_status_t uart_flush_buffer(uint8_t uart_id, const uart_flush_config_t *config);  /* ��մ��ڻ�����,�����ò��� */
uart_flush_status_t uart_flush(uint8_t uart_id);                                            /* ��մ��ڻ�����,ʹ��Ĭ������ */
int uart1_print(const char *str, uint16_t len);                                             /* ����1ֱ�Ӵ�ӡ�ַ��� */
int uart1_printf(const char *format, ...);                                                  /* ����1��ʽ����ӡ */
int uart3_printf(const char *format, ...);                                                  /* ����3��ʽ����ӡ */
int uart4_printf(const char *format, ...);                                                  /* ����4��ʽ����ӡ */
int uart5_printf(const char *format, ...);                                                  /* ����5��ʽ����ӡ */

#endif
