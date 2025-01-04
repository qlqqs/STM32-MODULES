/**
 * @brief ��������ģ�� - ����STM32F4��UART/DMA����
 * @details ��Ҫ����:
 *          1. ֧��UART1/3/4/5�മ�ڲ��й���
 *          2. ����FIFO���������շ�����
 *          3. DMA��ʽ�շ�,֧�ִ�����������
 *          4. ��̬����������, ���֧��1500000��1.5Mbps��
 *          5. ���������
 *          6. ��ʽ����ӡ
 * 
 * @note    ��ֲ˵��:
 *          1. �����ļ��޸�:
 *             - dev_uart.h: �޸�"��ֲ��Ҫ�޸ĵĺ궨��"
 *                           �޸�"��ֲ��Ҫ�޸ĵĺ�������"
 *             - bsp_uart.h: �޸�"��ֲ��Ҫ�޸ĵĺ궨��"
 *          2. Դ�ļ��޸�:
 *             - dev_uart.c: 
 *               > ����UART_HandleTypeDef, ����ʵ�ʴ���
 *               > ����"���ڻ���ģ��", ����ʵ�ʴ���
 *             - bsp_uart.c:
 *               > ������Ӧ��"UART_HandleTypeDef"��"DMA_HandleTypeDef", ����ʵ�ʴ���
 *               > ������������Ӧ��"case DEV_UARTx:", ����ʵ�ʴ���
 *          3. �ж�����:
 *             - stm32f4xx_it.c:
 *               > ��UART�ж��е���uart_dmarx_done_isr�Ⱥ���
 *               > ��DMA�ж��е���uart_dmatx_done_isr�Ⱥ���
 *               > ��IDLE�ж��е���uart_dmarx_idle_isr����
 *               > �����"stm32f4xx_it.c"
 * 
 * @usage   ����ʹ������:
 *          1. ��ʼ��: uart_device_init()
 *          2. ���ݷ���:
 *             - ԭʼ����: uartx_print() (x=1,3,4,5)
 *             - ��ʽ��: uartx_printf() (x=1,3,4,5)
 *          3. ���ݽ���:
 *             - uart_read()��FIFO��ȡ����
 *          4. ��������:
 *             - uart_set_baudrate()�޸Ĳ�����
 *             - uart_flush()��ս��ջ�����
 * 
 * @author  qlqqs
 * @date    2024.12.19
 */

#include "main.h"
#include "dev_uart.h"

/***************************** ��ֲ��Ҫ�޸ĵı��� ****************************/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* ���ڻ��� */
static uint8_t s_uart1_tx_buf[UART1_TX_BUF_SIZE];
static uint8_t s_uart1_rx_buf[UART1_RX_BUF_SIZE];
static uint8_t s_uart1_dmarx_buf[UART1_DMA_RX_BUF_SIZE] __attribute__((section(".ARM.__at_0x20000000")));
static uint8_t s_uart1_dmatx_buf[UART1_DMA_TX_BUF_SIZE] __attribute__((section(".ARM.__at_0x20000080")));

static uint8_t s_uart3_tx_buf[UART3_TX_BUF_SIZE];
static uint8_t s_uart3_rx_buf[UART3_RX_BUF_SIZE];
static uint8_t s_uart3_dmarx_buf[UART3_DMA_RX_BUF_SIZE];
static uint8_t s_uart3_dmatx_buf[UART3_DMA_TX_BUF_SIZE];

static uint8_t s_uart4_tx_buf[UART4_TX_BUF_SIZE];
static uint8_t s_uart4_rx_buf[UART4_RX_BUF_SIZE];
static uint8_t s_uart4_dmarx_buf[UART4_DMA_RX_BUF_SIZE];
static uint8_t s_uart4_dmatx_buf[UART4_DMA_TX_BUF_SIZE];

static uint8_t s_uart5_tx_buf[UART5_TX_BUF_SIZE];
static uint8_t s_uart5_rx_buf[UART5_RX_BUF_SIZE];
static uint8_t s_uart5_dmarx_buf[UART5_DMA_RX_BUF_SIZE];
static uint8_t s_uart5_dmatx_buf[UART5_DMA_TX_BUF_SIZE];
/***************************** ��ֲ��Ҫ�޸ĵı��� ****************************/

/* �����豸ʵ�� */
static uart_device_t s_uart_dev[UART_DEVICE_ARRAY_NUM] = {0};

/* ���� */
uint32_t s_UartTxRxCount[UART_COUNT_ARRAY_SIZE] = {0};

/* fifo�������� */
static void fifo_lock(void)
{
    __disable_irq();
}

/* fifo�������� */
static void fifo_unlock(void)
{
    __enable_irq();
}

/***************************** ��ֲ��Ҫ�޸ĵĺ��� ****************************/
/**
 * @brief �����豸��ʼ��
 * @param uart_id ����ID
 * @retval void
 */
void uart_device_init(uint8_t uart_id)
{
    if (uart_id == DEV_UART1)
    {
        /* ���ô���1�շ�fifo */
        fifo_register(&s_uart_dev[uart_id].tx_fifo, &s_uart1_tx_buf[0],
                      sizeof(s_uart1_tx_buf), NULL, NULL);
        fifo_register(&s_uart_dev[uart_id].rx_fifo, &s_uart1_rx_buf[0],
                      sizeof(s_uart1_rx_buf), fifo_lock, fifo_unlock);

        /* ���ô���1 DMA�շ�buf */
        s_uart_dev[uart_id].dmarx_buf = &s_uart1_dmarx_buf[0];
        s_uart_dev[uart_id].dmarx_buf_size = sizeof(s_uart1_dmarx_buf);
        s_uart_dev[uart_id].dmatx_buf = &s_uart1_dmatx_buf[0];
        s_uart_dev[uart_id].dmatx_buf_size = sizeof(s_uart1_dmatx_buf);
        uart_dma_init(uart_id, s_uart_dev[uart_id].dmarx_buf,
                     s_uart_dev[uart_id].dmarx_buf_size);/* ֻ�����ý���ģʽDMA,����ģʽ�跢������ʱ������ */
        s_uart_dev[uart_id].status  = 0;
    }
    else if (uart_id == DEV_UART3)
    {
        /* ���ô���3�շ�fifo */
        fifo_register(&s_uart_dev[uart_id].tx_fifo, &s_uart3_tx_buf[0],
                      sizeof(s_uart3_tx_buf), fifo_lock, fifo_unlock);
        fifo_register(&s_uart_dev[uart_id].rx_fifo, &s_uart3_rx_buf[0],
                      sizeof(s_uart3_rx_buf), fifo_lock, fifo_unlock);

        /* ���ô���3 DMA�շ�buf */
        s_uart_dev[uart_id].dmarx_buf = &s_uart3_dmarx_buf[0];
        s_uart_dev[uart_id].dmarx_buf_size = sizeof(s_uart3_dmarx_buf);
        s_uart_dev[uart_id].dmatx_buf = &s_uart3_dmatx_buf[0];
        s_uart_dev[uart_id].dmatx_buf_size = sizeof(s_uart3_dmatx_buf);
        uart_dma_init(uart_id, s_uart_dev[uart_id].dmarx_buf,
                     s_uart_dev[uart_id].dmarx_buf_size);
        s_uart_dev[uart_id].status  = 0;
    }
    else if (uart_id == DEV_UART4)
    {
        /* ���ô���4�շ�fifo */
        fifo_register(&s_uart_dev[uart_id].tx_fifo, &s_uart4_tx_buf[0],
                      sizeof(s_uart4_tx_buf), fifo_lock, fifo_unlock);
        fifo_register(&s_uart_dev[uart_id].rx_fifo, &s_uart4_rx_buf[0],
                      sizeof(s_uart4_rx_buf), fifo_lock, fifo_unlock);

        /* ���ô���4 DMA�շ�buf */
        s_uart_dev[uart_id].dmarx_buf = &s_uart4_dmarx_buf[0];
        s_uart_dev[uart_id].dmarx_buf_size = sizeof(s_uart4_dmarx_buf);
        s_uart_dev[uart_id].dmatx_buf = &s_uart4_dmatx_buf[0];
        s_uart_dev[uart_id].dmatx_buf_size = sizeof(s_uart4_dmatx_buf);
        uart_dma_init(uart_id, s_uart_dev[uart_id].dmarx_buf,
                     s_uart_dev[uart_id].dmarx_buf_size);
        s_uart_dev[uart_id].status  = 0;
    }
    else if (uart_id == DEV_UART5)
    {
        /* ���ô���5�շ�fifo */
        fifo_register(&s_uart_dev[uart_id].tx_fifo, &s_uart5_tx_buf[0],
                      sizeof(s_uart5_tx_buf), fifo_lock, fifo_unlock);
        fifo_register(&s_uart_dev[uart_id].rx_fifo, &s_uart5_rx_buf[0],
                      sizeof(s_uart5_rx_buf), fifo_lock, fifo_unlock);

        /* ���ô���5 DMA�շ�buf */
        s_uart_dev[uart_id].dmarx_buf = &s_uart5_dmarx_buf[0];
        s_uart_dev[uart_id].dmarx_buf_size = sizeof(s_uart5_dmarx_buf);
        s_uart_dev[uart_id].dmatx_buf = &s_uart5_dmatx_buf[0];
        s_uart_dev[uart_id].dmatx_buf_size = sizeof(s_uart5_dmatx_buf);
        uart_dma_init(uart_id, s_uart_dev[uart_id].dmarx_buf,
                     s_uart_dev[uart_id].dmarx_buf_size);
        s_uart_dev[uart_id].status  = 0;
    }       
}
/***************************** ��ֲ��Ҫ�޸ĵĺ��� ****************************/

/**
 * @brief  ���ڷ������ݽӿ�,ʵ����д�뷢��fifo,������dma����
 * @param uart_id ����ID
 * @param buf �������ݻ���
 * @param size �������ݴ�С
 * @retval ʵ��д����ֽ���
 */
uint16_t uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t size)
{
    return fifo_write(&s_uart_dev[uart_id].tx_fifo, buf, size);
}

/**
 * @brief  ���ڶ�ȡ���ݽӿ�,ʵ���Ǵӽ���fifo��ȡ
 * @param uart_id ����ID
 * @param buf �������ݻ���
 * @param size �������ݴ�С
 * @retval ʵ�ʶ�ȡ���ֽ���
 */
uint16_t uart_read(uint8_t uart_id, uint8_t *buf, uint16_t size)
{
    return fifo_read(&s_uart_dev[uart_id].rx_fifo, buf, size);
}

/**
 * @brief  ����dma��������жϴ���
 * @param uart_id ����ID
 * @retval void
 */
void uart_dmarx_done_isr(uint8_t uart_id)
{
    uint16_t recv_size;

    recv_size = s_uart_dev[uart_id].dmarx_buf_size - s_uart_dev[uart_id].last_dmarx_size;
    
    s_UartTxRxCount[uart_id * 2 + 1] += recv_size;
    fifo_write(&s_uart_dev[uart_id].rx_fifo,
               (const uint8_t *) & (s_uart_dev[uart_id].dmarx_buf[s_uart_dev[uart_id].last_dmarx_size]), recv_size);

    s_uart_dev[uart_id].last_dmarx_size = 0;
}

/**
 * @brief  ����dma���ջ����Сһ�������жϴ���
 * @param uart_id ����ID
 * @retval void
 */
void uart_dmarx_half_done_isr(uint8_t uart_id)
{
    uint16_t recv_total_size;
    uint16_t recv_size;

    recv_total_size = s_uart_dev[uart_id].dmarx_buf_size - bsp_uart_get_dmarx_buf_remain_size(uart_id);
    
    recv_size = recv_total_size - s_uart_dev[uart_id].last_dmarx_size;
    s_UartTxRxCount[uart_id * 2 + 1] += recv_size;

    fifo_write(&s_uart_dev[uart_id].rx_fifo,
               (const uint8_t *) & (s_uart_dev[uart_id].dmarx_buf[s_uart_dev[uart_id].last_dmarx_size]), recv_size);
    s_uart_dev[uart_id].last_dmarx_size = recv_total_size;
}

/**
 * @brief  ���ڿ����жϴ���
 * @param uart_id ����ID
 * @retval void
 */
void uart_dmarx_idle_isr(uint8_t uart_id)
{
    uint16_t recv_total_size;
    uint16_t recv_size;

    recv_total_size = s_uart_dev[uart_id].dmarx_buf_size - bsp_uart_get_dmarx_buf_remain_size(uart_id);
    
    recv_size = recv_total_size - s_uart_dev[uart_id].last_dmarx_size;
    s_UartTxRxCount[uart_id * 2 + 1] += recv_size;
    fifo_write(&s_uart_dev[uart_id].rx_fifo,
               (const uint8_t *) & (s_uart_dev[uart_id].dmarx_buf[s_uart_dev[uart_id].last_dmarx_size]), recv_size);
    s_uart_dev[uart_id].last_dmarx_size = recv_total_size;
}

/**
 * @brief  ����dma��������жϴ���
 * @param uart_id ����ID
 * @retval void
 */
void uart_dmatx_done_isr(uint8_t uart_id)
{
    s_uart_dev[uart_id].status = 0;	/* DMA���Ϳ��� */
}

/**
 * @brief  ѭ���Ӵ��ڷ���fifo��������,������dma���ͻ���,������dma����
 * @param uart_id ����ID
 * @retval void
 */
void uart_poll_dma_tx(uint8_t uart_id)
{
    uint16_t size = 0;

    if (0x01 == s_uart_dev[uart_id].status)
    {
        return;
    }
    size = fifo_read(&s_uart_dev[uart_id].tx_fifo, s_uart_dev[uart_id].dmatx_buf,
                     s_uart_dev[uart_id].dmatx_buf_size);
    if (size != 0)
    {
        s_UartTxRxCount[uart_id * 2 + 0] += size;

        s_uart_dev[uart_id].status = 0x01;	/* DMA����״̬ */
        bsp_uart_dmatx_config(uart_id, s_uart_dev[uart_id].dmatx_buf, size);
    }
}

/**
 * @brief  ���ô��ڲ�����
 * @param  uart_id: ����ID (DEV_UART1, DEV_UART3��)
 * @param  baudrate: Ҫ���õĲ�����ֵ
 * @retval UART_SET_BAUDRATE_OK: �ɹ�
 *         UART_SET_BAUDRATE_ERROR: ʧ��
 */
uint8_t uart_set_baudrate(uint8_t uart_id, uint32_t baudrate)
{
    UART_HandleTypeDef *huart = NULL;
    
    // ��ȡ��Ӧ���ھ��
    switch(uart_id) 
    {
        case DEV_UART1:
            huart = &huart1;
            break;
            
        case DEV_UART3:
            huart = &huart3;
            break;
            
        case DEV_UART4:
            huart = &huart4;
            break;
            
        case DEV_UART5:
            huart = &huart5;
            break;
            
        default:
            return UART_SET_BAUDRATE_ERROR;  // ��Ч�Ĵ���ID
    }
    
    // ֹͣ��ǰ��DMA����
    HAL_UART_DMAStop(huart);
    
    // ���²�����
    huart->Init.BaudRate = baudrate;
    if (HAL_UART_Init(huart) != HAL_OK)
    {
        return UART_SET_BAUDRATE_ERROR;
    }
    
    // ��������DMA����
    uart_device_init(uart_id);
    
    return UART_SET_BAUDRATE_OK;
}

/**
 * @brief  ��մ��ڻ�����
 * @param  uart_id: ����ID (DEV_UART1, DEV_UART3��)
 * @param  config: ������ò���,NULL��ʹ��Ĭ������
 * @retval uart_flush_status_t: ���״̬
 */
uart_flush_status_t uart_flush_buffer(uint8_t uart_id, const uart_flush_config_t *config)
{
    // Ĭ������
    static const uart_flush_config_t default_config = {
        .timeout_ms = 1000,  // Ĭ��1�볬ʱ
        .chunk_size = 64,    // Ĭ��ÿ�ζ�ȡ64�ֽ�
        .delay_ms = 1        // Ĭ��1ms��ʱ
    };
    
    // ʹ�ô�������û�Ĭ������
    const uart_flush_config_t *cfg = config ? config : &default_config;
    
    // ʹ��ջ�Ϸ���Ĺ̶���С������,���⶯̬�ڴ����
    uint8_t temp_buf[64];
    uint16_t read_size = cfg->chunk_size > sizeof(temp_buf) ? 
                        sizeof(temp_buf) : cfg->chunk_size;
    
    uint32_t start_time = HAL_GetTick();
    uint32_t total_bytes = 0;
    
    while(1) {
        // ��鳬ʱ
        if(HAL_GetTick() - start_time > cfg->timeout_ms) {
            return UART_FLUSH_TIMEOUT;
        }
        
        // ��ȡ����
        uint16_t len = uart_read(uart_id, temp_buf, read_size);
        if(len > 0) {
            total_bytes += len;
            HAL_Delay(cfg->delay_ms);
        } else {
            // û�и�������,������
            break;
        }
    }
    
    return UART_FLUSH_OK;
}

/**
 * @brief  ʹ��Ĭ�ϲ�����մ��ڻ�����
 * @param  uart_id: ����ID (DEV_UART1, DEV_UART3��)
 * @retval uart_flush_status_t: ���״̬
 */
uart_flush_status_t uart_flush(uint8_t uart_id)
{
    return uart_flush_buffer(uart_id, NULL);
}

/***************************** ��ֲ��Ҫ�޸ĵĺ��� ****************************/
 * @brief  ����1ֱ�����(����ʽ��)����
 * @param  str: Ҫ������ַ���
 * @param  len: Ҫ������ַ�������
 * @retval ʵ�ʷ��͵��ֽ���
 */
int uart1_print(const char *str, uint16_t len)
{
    int ret;
    
    // ֱ��ʹ��uart_write��������
    ret = uart_write(DEV_UART1, (uint8_t*)str, len);
    
    // ����DMA����
    uart_poll_dma_tx(DEV_UART1);
    
    return ret;
}

/**
 * @brief  ����1��ʽ���������
 * @param  format: ��ʽ���ַ���
 * @param  ...: �ɱ����
 * @retval ʵ�ʷ��͵��ֽ���
 */
int uart1_printf(const char *format, ...)
{
    va_list args;
    uint8_t buf[512];  // ���󻺳�����֧�ָ���������
    int length;
    uint16_t ret;

    // ��ʽ���ַ���
    va_start(args, format);
    length = vsnprintf((char *)buf, sizeof(buf), format, args);
    va_end(args);

    // ��黺�����Ƿ����
    if (length < 0 || length >= sizeof(buf)) {
        return 0;
    }

    // ���͸�ʽ���������
    ret = uart_write(DEV_UART1, buf, length);
    
    // ����DMA����
    uart_poll_dma_tx(DEV_UART1);
    
    return ret;
}

/**
 * @brief  ����3ֱ�����(����ʽ��)����
 * @param  str: Ҫ������ַ���
 * @param  len: Ҫ������ַ�������
 * @retval ʵ�ʷ��͵��ֽ���
 */
int uart3_print(const char *str, uint16_t len)
{
    int ret;
    
    // ֱ��ʹ��uart_write��������
    ret = uart_write(DEV_UART3, (uint8_t*)str, len);
    
    // ����DMA����
    uart_poll_dma_tx(DEV_UART3);
    
    return ret;
}

/**
 * @brief  ����3��ʽ���������
 * @param  format: ��ʽ���ַ���
 * @param  ...: �ɱ����
 * @retval ʵ�ʷ��͵��ֽ���
 */
int uart3_printf(const char *format, ...)
{
    va_list args;
    uint8_t buf[512];  // ���󻺳�����֧�ָ���������
    int length;
    uint16_t ret;

    // ��ʽ���ַ���
    va_start(args, format);
    length = vsnprintf((char *)buf, sizeof(buf), format, args);
    va_end(args);

    // ��黺�����Ƿ����
    if (length < 0 || length >= sizeof(buf)) {
        return 0;
    }

    // ���͸�ʽ���������
    ret = uart_write(DEV_UART3, buf, length);
    
    // ����DMA����
    uart_poll_dma_tx(DEV_UART3);
    
    return ret;
}

/**
 * @brief  ����4ֱ�����(����ʽ��)����
 * @param  str: Ҫ������ַ���
 * @param  len: Ҫ������ַ�������
 * @retval ʵ�ʷ��͵��ֽ���
 */
int uart4_print(const char *str, uint16_t len)
{
    int ret;
    
    // ֱ��ʹ��uart_write��������
    ret = uart_write(DEV_UART4, (uint8_t*)str, len);
    
    // ����DMA����
    uart_poll_dma_tx(DEV_UART4);
    
    return ret;
}

/**
 * @brief  ����4��ʽ���������
 * @param  format: ��ʽ���ַ���
 * @param  ...: �ɱ����
 * @retval ʵ�ʷ��͵��ֽ���
 */
int uart4_printf(const char *format, ...)
{
    va_list args;
    uint8_t buf[512];  // ���󻺳�����֧�ָ���������
    int length;
    uint16_t ret;

    // ��ʽ���ַ���
    va_start(args, format);
    length = vsnprintf((char *)buf, sizeof(buf), format, args);
    va_end(args);

    // ��黺�����Ƿ����
    if (length < 0 || length >= sizeof(buf)) {
        return 0;
    }

    // ���͸�ʽ���������
    ret = uart_write(DEV_UART4, buf, length);
    
    // ����DMA����
    uart_poll_dma_tx(DEV_UART4);
    
    return ret;
}

/**
 * @brief  ����5ֱ�����(����ʽ��)����
 * @param  str: Ҫ������ַ���
 * @param  len: Ҫ������ַ�������
 * @retval ʵ�ʷ��͵��ֽ���
 */
int uart5_print(const char *str, uint16_t len)
{
    int ret;
    
    // ֱ��ʹ��uart_write��������
    ret = uart_write(DEV_UART5, (uint8_t*)str, len);
    
    // ����DMA����
    uart_poll_dma_tx(DEV_UART5);
    
    return ret;
}

/**
 * @brief  ����5��ʽ���������
 * @param  format: ��ʽ���ַ���
 * @param  ...: �ɱ����
 * @retval ʵ�ʷ��͵��ֽ���
 */
int uart5_printf(const char *format, ...)
{
    va_list args;
    uint8_t buf[512];  // ���󻺳�����֧�ָ���������
    int length;
    uint16_t ret;

    // ��ʽ���ַ���
    va_start(args, format);
    length = vsnprintf((char *)buf, sizeof(buf), format, args);
    va_end(args);

    // ��黺�����Ƿ����
    if (length < 0 || length >= sizeof(buf)) {
        return 0;
    }

    // ���͸�ʽ���������
    ret = uart_write(DEV_UART5, buf, length);
    
    // ����DMA����
    uart_poll_dma_tx(DEV_UART5);
    
    return ret;
}
/***************************** ��ֲ��Ҫ�޸ĵĺ��� ****************************/
