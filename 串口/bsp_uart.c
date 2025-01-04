#include "main.h"
#include "bsp_uart.h"
#include "stm32f4xx_hal.h"

/***************************** ��ֲ��Ҫ�޸ĵı��� ****************************/
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
/***************************** ��ֲ��Ҫ�޸ĵı��� ****************************/

/***************************** ��ֲ��Ҫ�޸ĵĺ��� ****************************/
/**
 * @brief DMA��������UART����
 * @param uart_id ����ID
 * @param mem_addr ���ͻ����ַ
 * @param mem_size ���ͻ����С
 * @retval void
 */
void bsp_uart_dmatx_config(uint8_t uart_id, uint8_t *mem_addr, uint32_t mem_size)
{
    switch(uart_id) {
        case DEV_UART1:
            /* ֹͣDMA���� */
            HAL_DMA_Abort(&hdma_usart1_tx);
            /* ����DMA�ڴ��ַ�ʹ��䳤�� */
            HAL_DMA_Start_IT(&hdma_usart1_tx, (uint32_t)mem_addr, (uint32_t)&huart1.Instance->DR, mem_size);
            /* ʹ��UART��DMA���� */
            SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
            break;
            
        case DEV_UART3:
            HAL_DMA_Abort(&hdma_usart3_tx);
            HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)mem_addr, (uint32_t)&huart3.Instance->DR, mem_size);
            SET_BIT(huart3.Instance->CR3, USART_CR3_DMAT);
            break;
            
        case DEV_UART4:
            HAL_DMA_Abort(&hdma_uart4_tx);
            HAL_DMA_Start_IT(&hdma_uart4_tx, (uint32_t)mem_addr, (uint32_t)&huart4.Instance->DR, mem_size);
            SET_BIT(huart4.Instance->CR3, USART_CR3_DMAT);
            break;
            
        case DEV_UART5:
            HAL_DMA_Abort(&hdma_uart5_tx);
            HAL_DMA_Start_IT(&hdma_uart5_tx, (uint32_t)mem_addr, (uint32_t)&huart5.Instance->DR, mem_size);
            SET_BIT(huart5.Instance->CR3, USART_CR3_DMAT);
            break;
    }
}

/**
 * @brief DMA��������UART����
 * @param uart_id ����ID
 * @param mem_addr ���ջ����ַ
 * @param mem_size ���ջ����С
 * @retval void
 */
void bsp_uart_dmarx_config(uint8_t uart_id, uint8_t *mem_addr, uint32_t mem_size)
{
    switch(uart_id) {
        case DEV_UART1:
            /* ֹͣDMA���� */
            HAL_DMA_Abort(&hdma_usart1_rx);
            /* ����DMA�ڴ��ַ�ʹ��䳤�� */
            HAL_DMA_Start_IT(&hdma_usart1_rx, (uint32_t)&huart1.Instance->DR, (uint32_t)mem_addr, mem_size);
            /* ʹ��UART��DMA���� */
            SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
            break;
            
        case DEV_UART3:
            HAL_DMA_Abort(&hdma_usart3_rx);
            HAL_DMA_Start_IT(&hdma_usart3_rx, (uint32_t)&huart3.Instance->DR, (uint32_t)mem_addr, mem_size);
            SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
            break;
            
        case DEV_UART4:
            HAL_DMA_Abort(&hdma_uart4_rx);
            HAL_DMA_Start_IT(&hdma_uart4_rx, (uint32_t)&huart4.Instance->DR, (uint32_t)mem_addr, mem_size);
            SET_BIT(huart4.Instance->CR3, USART_CR3_DMAR);
            break;
            
        case DEV_UART5:
            HAL_DMA_Abort(&hdma_uart5_rx);
            HAL_DMA_Start_IT(&hdma_uart5_rx, (uint32_t)&huart5.Instance->DR, (uint32_t)mem_addr, mem_size);
            SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
            break;
    }
}

/**
 * @brief UART DMA ��ʼ��
 * @param uart_id ����ID
 * @param mem_addr ���ջ����ַ
 * @param mem_size ���ջ����С
 * @retval void
 */
void uart_dma_init(uint8_t uart_id, uint8_t *mem_addr, uint32_t mem_size)
{
    /* ����DMA���� */
    bsp_uart_dmarx_config(uart_id, mem_addr, mem_size);

    /* ʹ��DMA����Ϳ����ж� */
    switch(uart_id) {
        case DEV_UART1:
            __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
            break;
        case DEV_UART3:
            __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); 
            break;
        case DEV_UART4:
            __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
            break;
        case DEV_UART5:
            __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
            break;
    }
}


/**
 * @brief ��ȡUART DMA���ջ�����ʣ���С
 * @param uart_id ����ID
 * @retval ʣ���С
 */
uint16_t bsp_uart_get_dmarx_buf_remain_size(uint8_t uart_id)
{
    switch(uart_id) {
        case DEV_UART1:
            return (uint16_t)(huart1.hdmarx->Instance->NDTR);
        case DEV_UART3:
            return (uint16_t)(huart3.hdmarx->Instance->NDTR);
        case DEV_UART4:
            return (uint16_t)(huart4.hdmarx->Instance->NDTR);
        case DEV_UART5:
            return (uint16_t)(huart5.hdmarx->Instance->NDTR);
        default:
            return 0;
    }
}
/***************************** ��ֲ��Ҫ�޸ĵĺ��� ****************************/
