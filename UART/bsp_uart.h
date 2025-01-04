
#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include <stdint.h>
#include <stdbool.h>

/************************************** ����Ϊ�����궨�� ******************************************/
/******************* ��ֲ��Ҫ�޸ĵĺ궨�� ******************/
/* �����豸ID���� */
#define DEV_UART1                1
#define DEV_UART3                3
#define DEV_UART4                4
#define DEV_UART5                5

/* �����豸�������� */
#define UART_DEVICE_MAX_NUM      5                                 /* ��󴮿��豸���� �������ΪUART5*/
#define UART_DEVICE_ARRAY_NUM    (UART_DEVICE_MAX_NUM + 1)         /* �����豸�����С UART_DEVICE_MAX_NUM + 1*/
#define UART_COUNT_ARRAY_SIZE    ((UART_DEVICE_MAX_NUM + 1) * 2)   /* �����շ����������С 2*UART_DEVICE_MAX_NUM */
/******************* ��ֲ��Ҫ�޸ĵĺ궨�� ******************/

/* �������ò�����״̬���� */
#define UART_SET_BAUDRATE_OK     0                                 /* ���ò����ʳɹ� */
#define UART_SET_BAUDRATE_ERROR  1                                 /* ���ò�����ʧ�� */

/************************************** ����Ϊ�������� ******************************************/
/* �ⲿ�������� */
void bsp_uart_dmatx_config(uint8_t uart_id, uint8_t *mem_addr, uint32_t mem_size);
void bsp_uart_dmarx_config(uint8_t uart_id, uint8_t *mem_addr, uint32_t mem_size);
void uart_dma_init(uint8_t uart_id, uint8_t *mem_addr, uint32_t mem_size);
uint16_t bsp_uart_get_dmarx_buf_remain_size(uint8_t uart_id);

#endif /* _BSP_UART_H_ */
