/**
 ****************************************************************************************************
 * Date 2024.12.19 qlqqs
 * atk_ble03 蓝牙模块驱动 资料和例程存在问题，尽管历程能正常运行，但例程中LD0和STA应该是反过来的！
 * 模块的STA引脚应该推挽输出高电平，LED引脚应该输入且Pull down！
 ****************************************************************************************************
 */

#ifndef __ATK_BLE03_H
#define __ATK_BLE03_H

#include <stdint.h>

/************************************** 以下为参数宏定义 ******************************************/
/* 蓝牙串口定义 */
#define ATK_BLE03_UART                      DEV_UART3    /* 蓝牙串口，依赖于bsp_uart.h */ 

/* 引脚定义 */
#define ATK_BLE03_LDO_GPIO_PORT             BLE_LED_GPIO_Port
#define ATK_BLE03_LDO_GPIO_PIN              BLE_LED_Pin
#define ATK_BLE03_LDO_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOI_CLK_ENABLE(); }while(0)

#define ATK_BLE03_STA_GPIO_PORT             BLE_EN_GPIO_Port
#define ATK_BLE03_STA_GPIO_PIN              BLE_EN_Pin
#define ATK_BLE03_STA_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/* IO操作 */
#define ATK_BLE03_READ_STA()                HAL_GPIO_ReadPin(ATK_BLE03_STA_GPIO_PORT, ATK_BLE03_STA_GPIO_PIN)
#define ATK_BLE03_LDO(x)                    do{ x ?                                                                                     \
                                                HAL_GPIO_WritePin(ATK_BLE03_LDO_GPIO_PORT, ATK_BLE03_LDO_GPIO_PIN, GPIO_PIN_SET) :    \
                                                HAL_GPIO_WritePin(ATK_BLE03_LDO_GPIO_PORT, ATK_BLE03_LDO_GPIO_PIN, GPIO_PIN_RESET);   \
                                            }while(0)

/* 波特率范围 */
#define ATK_BLE03_BAUD_MIN                  1
#define ATK_BLE03_BAUD_MAX                  16

/* 错误代码 */
#define ATK_BLE03_EOK      0                /* 没有错误 */
#define ATK_BLE03_ERROR    1                /* 错误 */
#define ATK_BLE03_ETIMEOUT 2                /* 超时错误 */
#define ATK_BLE03_EINVAL   3                /* 参数错误 */

/************************************** 以下为结构体声明 ******************************************/
/* ATK-BLE03状态枚举 */
typedef enum {
    ATK_BLE03_DISABLE = 0,                  /* 蓝牙关闭 */
    ATK_BLE03_ENABLE = 1                    /* 蓝牙开启 */
} atk_ble03_state_t;

/* ATK-BLE03连接状态枚举 */
typedef enum
{
    ATK_BLE03_CONNECTED = 0x00,             /* 已连接 */
    ATK_BLE03_DISCONNECTED,                 /* 未连接 */
} atk_ble03_conn_sta_t;

/* 串口波特率枚举 */
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

/************************************** 以下为函数声明 ******************************************/
/* 操作函数 */
void atk_ble03_init(void);                                                   /* ATK-BLE03初始化 */
void atk_ble03(atk_ble03_state_t state);                                     /* ATK-BLE03初始化 */
atk_ble03_conn_sta_t atk_ble03_get_conn_sta(void);                           /* 获取ATK-BLE03连接状态 */
uint8_t atk_ble03_send_at_cmd(char *cmd, char *ack, uint32_t timeout);       /* ATK-BLE03发送AT指令 */
uint8_t atk_ble03_set_uart(void);


#endif
