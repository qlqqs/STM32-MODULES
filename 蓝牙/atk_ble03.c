/**
 ****************************************************************************************************
 * @file        atk_ble03.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-01
 * @brief       ATK-BLE03模块驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 阿波罗 F429开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "atk_ble03.h"
#include "dev_uart.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* 串口波特率 */
static uint8_t baud_num = 8;

/**
 * @brief       ATK-BLE03初始化
 * @param       无
 * @retval      无
 */
void atk_ble03_init(void)
{
    atk_ble03(ATK_BLE03_ENABLE);
}

/**
 * @brief       ATK-BLE03启动/关闭
 * @param       state: ATK_BLE03_ENABLE 启动, ATK_BLE03_DISABLE 关闭
 * @retval      无
 */
void atk_ble03(atk_ble03_state_t state)
{
    /* 使能/禁用ATK-BLE03模块 */
    ATK_BLE03_LDO(state);
    
    if (state == ATK_BLE03_ENABLE)
    {
        /* 启动时等待模块稳定 */
        HAL_Delay(200);
        uart_flush(ATK_BLE03_UART);
    }
}

/**
 * @brief       获取ATK-BLE03连接状态
 * @param       无
 * @retval      ATK_BLE03_CONNECTED   : 已连接
 *              ATK_BLE03_DISCONNECTED: 未连接
 */
atk_ble03_conn_sta_t atk_ble03_get_conn_sta(void)
{
    if (ATK_BLE03_READ_STA() != 0)
    {
        return ATK_BLE03_CONNECTED;
    }
    
    return ATK_BLE03_DISCONNECTED;
}

/**
 * @brief       ATK-BLE03发送AT指令
 * @param       cmd    : 待发送的AT指令
 *              ack    : 等待的响应
 *              timeout: 等待超时时间
 * @retval      ATK_BLE03_EOK     : 函数执行成功
 *              ATK_BLE03_ETIMEOUT: 等待期望应答超时，函数执行失败
 */
uint8_t atk_ble03_send_at_cmd(char *cmd, char *ack, uint32_t timeout)
{
    uint8_t ret_buf[128];
    uint16_t len;
    
    // 清空接收缓冲区
    uart_flush_config_t flush_config = {
        .timeout_ms = 100,
        .chunk_size = 64,
        .delay_ms = 1
    };
    uart_flush_buffer(DEV_UART3, &flush_config);
    
    // 发送AT指令
    uart3_printf("%s\r\n", cmd);
    
    if ((ack == NULL) || (timeout == 0))
    {
        return ATK_BLE03_EOK;
    }
    
    // 等待响应
    while (timeout > 0)
    {
        len = uart_read(DEV_UART3, ret_buf, sizeof(ret_buf)-1);
        if (len > 0)
        {
            ret_buf[len] = '\0';
            if (strstr((const char *)ret_buf, ack) != NULL)
            {
                return ATK_BLE03_EOK;
            }
        }
        timeout--;
        HAL_Delay(1);
    }
    
    return ATK_BLE03_ETIMEOUT;
}
/**
 * @brief       设置ATK-BLE03串口参数
 * @param       baudrate: 波特率
 * @retval      ATK_BLE03_EOK   : 设置ATK-BLE03串口波特率成功
 *              ATK_BLE03_ERROR : 设置ATK-BLE03串口波特率失败
 *              ATK_BLE03_EINVAL: 函数参数错误
 */
uint8_t atk_ble03_set_uart(void)
{
    uint32_t _baudrate;
    char cmd[32];  // AT指令缓冲区
    
    // 波特率循环切换
    if(baud_num >= ATK_BLE03_BAUD_MIN && baud_num <= ATK_BLE03_BAUD_MAX)
    {
        baud_num++;
    }
    else
    {
        baud_num = ATK_BLE03_BAUD_MIN;
    }

    switch (baud_num)
    {
        case ATK_BLE03_UART_BAUDRATE_2400:
        {
            _baudrate = 2400;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_4800:
        {
            _baudrate = 4800;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_9600:
        {
            _baudrate = 9600;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_19200:
        {
            _baudrate = 19200;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_31250:
        {
            _baudrate = 31250;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_38400:
        {
            _baudrate = 38400;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_57600:
        {
            _baudrate = 57600;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_115200:
        {
            _baudrate = 115200;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_230400:
        {
            _baudrate = 230400;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_256000:
        {
            _baudrate = 256000;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_460800:
        {
            _baudrate = 460800;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_512000:
        {
            _baudrate = 512000;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_750000:
        {
            _baudrate = 750000;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_800000:
        {
            _baudrate = 800000;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_921600:
        {
            _baudrate = 921600;
            break;
        }
        case ATK_BLE03_UART_BAUDRATE_1000000:
        {
            _baudrate = 1000000;
            break;
        }
        default:
        {
            return ATK_BLE03_EINVAL;
        }
    }

    // 构造AT指令
    sprintf(cmd, "AT+BAUD=%d", _baudrate);
    
    // 发送AT指令并等待响应
    if (atk_ble03_send_at_cmd(cmd, "OK", 100) != ATK_BLE03_EOK)
    {
        return ATK_BLE03_ERROR;
    }
    
    // 更新串口波特率
    uart_set_baudrate(DEV_UART3, _baudrate);
    
    return ATK_BLE03_EOK;
}
