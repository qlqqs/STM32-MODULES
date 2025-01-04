/**
 ****************************************************************************************************
 * @file        atk_ble03.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-01
 * @brief       ATK-BLE03ģ����������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ������ F429������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "atk_ble03.h"
#include "dev_uart.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* ���ڲ����� */
static uint8_t baud_num = 8;

/**
 * @brief       ATK-BLE03��ʼ��
 * @param       ��
 * @retval      ��
 */
void atk_ble03_init(void)
{
    atk_ble03(ATK_BLE03_ENABLE);
}

/**
 * @brief       ATK-BLE03����/�ر�
 * @param       state: ATK_BLE03_ENABLE ����, ATK_BLE03_DISABLE �ر�
 * @retval      ��
 */
void atk_ble03(atk_ble03_state_t state)
{
    /* ʹ��/����ATK-BLE03ģ�� */
    ATK_BLE03_LDO(state);
    
    if (state == ATK_BLE03_ENABLE)
    {
        /* ����ʱ�ȴ�ģ���ȶ� */
        HAL_Delay(200);
        uart_flush(ATK_BLE03_UART);
    }
}

/**
 * @brief       ��ȡATK-BLE03����״̬
 * @param       ��
 * @retval      ATK_BLE03_CONNECTED   : ������
 *              ATK_BLE03_DISCONNECTED: δ����
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
 * @brief       ATK-BLE03����ATָ��
 * @param       cmd    : �����͵�ATָ��
 *              ack    : �ȴ�����Ӧ
 *              timeout: �ȴ���ʱʱ��
 * @retval      ATK_BLE03_EOK     : ����ִ�гɹ�
 *              ATK_BLE03_ETIMEOUT: �ȴ�����Ӧ��ʱ������ִ��ʧ��
 */
uint8_t atk_ble03_send_at_cmd(char *cmd, char *ack, uint32_t timeout)
{
    uint8_t ret_buf[128];
    uint16_t len;
    
    // ��ս��ջ�����
    uart_flush_config_t flush_config = {
        .timeout_ms = 100,
        .chunk_size = 64,
        .delay_ms = 1
    };
    uart_flush_buffer(DEV_UART3, &flush_config);
    
    // ����ATָ��
    uart3_printf("%s\r\n", cmd);
    
    if ((ack == NULL) || (timeout == 0))
    {
        return ATK_BLE03_EOK;
    }
    
    // �ȴ���Ӧ
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
 * @brief       ����ATK-BLE03���ڲ���
 * @param       baudrate: ������
 * @retval      ATK_BLE03_EOK   : ����ATK-BLE03���ڲ����ʳɹ�
 *              ATK_BLE03_ERROR : ����ATK-BLE03���ڲ�����ʧ��
 *              ATK_BLE03_EINVAL: ������������
 */
uint8_t atk_ble03_set_uart(void)
{
    uint32_t _baudrate;
    char cmd[32];  // ATָ�����
    
    // ������ѭ���л�
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

    // ����ATָ��
    sprintf(cmd, "AT+BAUD=%d", _baudrate);
    
    // ����ATָ��ȴ���Ӧ
    if (atk_ble03_send_at_cmd(cmd, "OK", 100) != ATK_BLE03_EOK)
    {
        return ATK_BLE03_ERROR;
    }
    
    // ���´��ڲ�����
    uart_set_baudrate(DEV_UART3, _baudrate);
    
    return ATK_BLE03_EOK;
}
