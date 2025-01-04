/**
 ****************************************************************************************************
 * Date 2024.12.19 qlqqs
 * atk_ble03 ����ģ������ ���Ϻ����̴������⣬�����������������У���������LD0��STAӦ���Ƿ������ģ�
 * ģ���STA����Ӧ����������ߵ�ƽ��LED����Ӧ��������Pull down��
 ****************************************************************************************************
 */

#ifndef __ATK_BLE03_H
#define __ATK_BLE03_H

#include <stdint.h>

/************************************** ����Ϊ�����궨�� ******************************************/
/* �������ڶ��� */
#define ATK_BLE03_UART                      DEV_UART3    /* �������ڣ�������bsp_uart.h */ 

/* ���Ŷ��� */
#define ATK_BLE03_LDO_GPIO_PORT             BLE_LED_GPIO_Port
#define ATK_BLE03_LDO_GPIO_PIN              BLE_LED_Pin
#define ATK_BLE03_LDO_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)

#define ATK_BLE03_STA_GPIO_PORT             BLE_EN_GPIO_Port
#define ATK_BLE03_STA_GPIO_PIN              BLE_EN_Pin
#define ATK_BLE03_STA_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/* IO���� */
#define ATK_BLE03_READ_STA()                HAL_GPIO_ReadPin(ATK_BLE03_STA_GPIO_PORT, ATK_BLE03_STA_GPIO_PIN)
#define ATK_BLE03_LDO(x)                    do{ x ?                                                                                     \
                                                HAL_GPIO_WritePin(ATK_BLE03_LDO_GPIO_PORT, ATK_BLE03_LDO_GPIO_PIN, GPIO_PIN_SET) :    \
                                                HAL_GPIO_WritePin(ATK_BLE03_LDO_GPIO_PORT, ATK_BLE03_LDO_GPIO_PIN, GPIO_PIN_RESET);   \
                                            }while(0)

/* �����ʷ�Χ */
#define ATK_BLE03_BAUD_MIN                  1
#define ATK_BLE03_BAUD_MAX                  16

/* ������� */
#define ATK_BLE03_EOK      0                /* û�д��� */
#define ATK_BLE03_ERROR    1                /* ���� */
#define ATK_BLE03_ETIMEOUT 2                /* ��ʱ���� */
#define ATK_BLE03_EINVAL   3                /* �������� */

/************************************** ����Ϊ�ṹ������ ******************************************/
/* ATK-BLE03״̬ö�� */
typedef enum {
    ATK_BLE03_DISABLE = 0,                  /* �����ر� */
    ATK_BLE03_ENABLE = 1                    /* �������� */
} atk_ble03_state_t;

/* ATK-BLE03����״̬ö�� */
typedef enum
{
    ATK_BLE03_CONNECTED = 0x00,             /* ������ */
    ATK_BLE03_DISCONNECTED,                 /* δ���� */
} atk_ble03_conn_sta_t;

/* ���ڲ�����ö�� */
typedef enum
{

    ATK_BLE03_UART_BAUDRATE_2400   = 0x01,                   /* 2400    */
    ATK_BLE03_UART_BAUDRATE_4800,                            /* 4800    */
    ATK_BLE03_UART_BAUDRATE_9600,                            /* 9600    */
    ATK_BLE03_UART_BAUDRATE_19200,                           /* 19200   */
    ATK_BLE03_UART_BAUDRATE_31250,                           /* 31250   */
    ATK_BLE03_UART_BAUDRATE_38400,                           /* 38400   */
    ATK_BLE03_UART_BAUDRATE_57600,                           /* 57600   */
    ATK_BLE03_UART_BAUDRATE_115200,                          /* 115200  */
    ATK_BLE03_UART_BAUDRATE_230400,                          /* 230400  */
    ATK_BLE03_UART_BAUDRATE_256000,                          /* 256000  */
    ATK_BLE03_UART_BAUDRATE_460800,                          /* 460800  */
    ATK_BLE03_UART_BAUDRATE_512000,                          /* 512000  */
    ATK_BLE03_UART_BAUDRATE_750000,                          /* 750000  */
    ATK_BLE03_UART_BAUDRATE_800000,                          /* 800000  */
    ATK_BLE03_UART_BAUDRATE_921600,                          /* 921600  */
    ATK_BLE03_UART_BAUDRATE_1000000,                         /* 1000000 */
} atk_ble03_uart_baudrate_t;

/************************************** ����Ϊ�������� ******************************************/
/* �������� */
void atk_ble03_init(void);                                                   /* ATK-BLE03��ʼ�� */
void atk_ble03(atk_ble03_state_t state);                                     /* ATK-BLE03��ʼ�� */
atk_ble03_conn_sta_t atk_ble03_get_conn_sta(void);                           /* ��ȡATK-BLE03����״̬ */
uint8_t atk_ble03_send_at_cmd(char *cmd, char *ack, uint32_t timeout);       /* ATK-BLE03����ATָ�� */
uint8_t atk_ble03_set_uart(void);


#endif
