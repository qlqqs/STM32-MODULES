/**
 * @brief 串口驱动模块
 * @details 支持以下功能:
 *          1. 支持多个串口同时工作
 *          2. 使用FIFO缓冲区管理收发数据
 *          3. 支持DMA方式收发数据
 *          4. 支持串口波特率动态配置
 *          5. 支持缓冲区清空功能
 *          6. 支持格式化打印功能
 * @note    移植时需要:
 *          1. 修改串口缓存大小定义(dev_uart.h中的UART_xx_BUF_SIZE)
 *          2. 修改串口设备实例(dev_uart.c中的s_uart_dev)
 *          3. 修改串口句柄(dev_uart.c中的huartx)
 *          4. 在中断服务程序中调用uart_dmarx_done_isr()等函数
 * @author  qlqqs
 * @date    2024.12.19
 */

#include "main.h"
#include "dev_uart.h"

/***************************** 移植需要修改的变量 ****************************/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* 串口缓存 */
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

/* 串口设备实例 */
static uart_device_t s_uart_dev[UART_DEVICE_ARRAY_NUM] = {0};

/* 测试 */
uint32_t s_UartTxRxCount[UART_COUNT_ARRAY_SIZE] = {0};
/***************************** 移植需要修改的变量 ****************************/

/* fifo上锁函数 */
static void fifo_lock(void)
{
    __disable_irq();
}

/* fifo解锁函数 */
static void fifo_unlock(void)
{
    __enable_irq();
}

/***************************** 移植需要修改的函数 ****************************/
/**
 * @brief 串口设备初始化
 * @param uart_id 串口ID
 * @retval void
 */
void uart_device_init(uint8_t uart_id)
{
    if (uart_id == DEV_UART1)
    {
        /* 配置串口1收发fifo */
        fifo_register(&s_uart_dev[uart_id].tx_fifo, &s_uart1_tx_buf[0],
                      sizeof(s_uart1_tx_buf), NULL, NULL);
        fifo_register(&s_uart_dev[uart_id].rx_fifo, &s_uart1_rx_buf[0],
                      sizeof(s_uart1_rx_buf), fifo_lock, fifo_unlock);

        /* 配置串口1 DMA收发buf */
        s_uart_dev[uart_id].dmarx_buf = &s_uart1_dmarx_buf[0];
        s_uart_dev[uart_id].dmarx_buf_size = sizeof(s_uart1_dmarx_buf);
        s_uart_dev[uart_id].dmatx_buf = &s_uart1_dmatx_buf[0];
        s_uart_dev[uart_id].dmatx_buf_size = sizeof(s_uart1_dmatx_buf);
        uart_dma_init(uart_id, s_uart_dev[uart_id].dmarx_buf,
                     s_uart_dev[uart_id].dmarx_buf_size);/* 只需配置接收模式DMA,发送模式需发送数据时才配置 */
        s_uart_dev[uart_id].status  = 0;
    }
    else if (uart_id == DEV_UART3)
    {
        /* 配置串口3收发fifo */
        fifo_register(&s_uart_dev[uart_id].tx_fifo, &s_uart3_tx_buf[0],
                      sizeof(s_uart3_tx_buf), fifo_lock, fifo_unlock);
        fifo_register(&s_uart_dev[uart_id].rx_fifo, &s_uart3_rx_buf[0],
                      sizeof(s_uart3_rx_buf), fifo_lock, fifo_unlock);

        /* 配置串口3 DMA收发buf */
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
        /* 配置串口4收发fifo */
        fifo_register(&s_uart_dev[uart_id].tx_fifo, &s_uart4_tx_buf[0],
                      sizeof(s_uart4_tx_buf), fifo_lock, fifo_unlock);
        fifo_register(&s_uart_dev[uart_id].rx_fifo, &s_uart4_rx_buf[0],
                      sizeof(s_uart4_rx_buf), fifo_lock, fifo_unlock);

        /* 配置串口4 DMA收发buf */
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
        /* 配置串口5收发fifo */
        fifo_register(&s_uart_dev[uart_id].tx_fifo, &s_uart5_tx_buf[0],
                      sizeof(s_uart5_tx_buf), fifo_lock, fifo_unlock);
        fifo_register(&s_uart_dev[uart_id].rx_fifo, &s_uart5_rx_buf[0],
                      sizeof(s_uart5_rx_buf), fifo_lock, fifo_unlock);

        /* 配置串口5 DMA收发buf */
        s_uart_dev[uart_id].dmarx_buf = &s_uart5_dmarx_buf[0];
        s_uart_dev[uart_id].dmarx_buf_size = sizeof(s_uart5_dmarx_buf);
        s_uart_dev[uart_id].dmatx_buf = &s_uart5_dmatx_buf[0];
        s_uart_dev[uart_id].dmatx_buf_size = sizeof(s_uart5_dmatx_buf);
        uart_dma_init(uart_id, s_uart_dev[uart_id].dmarx_buf,
                     s_uart_dev[uart_id].dmarx_buf_size);
        s_uart_dev[uart_id].status  = 0;
    }       
}
/***************************** 移植需要修改的函数 ****************************/

/**
 * @brief  串口发送数据接口,实际是写入发送fifo,发送由dma处理
 * @param uart_id 串口ID
 * @param buf 发送数据缓存
 * @param size 发送数据大小
 * @retval 实际写入的字节数
 */
uint16_t uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t size)
{
    return fifo_write(&s_uart_dev[uart_id].tx_fifo, buf, size);
}

/**
 * @brief  串口读取数据接口,实际是从接收fifo读取
 * @param uart_id 串口ID
 * @param buf 接收数据缓存
 * @param size 接收数据大小
 * @retval 实际读取的字节数
 */
uint16_t uart_read(uint8_t uart_id, uint8_t *buf, uint16_t size)
{
    return fifo_read(&s_uart_dev[uart_id].rx_fifo, buf, size);
}

/**
 * @brief  串口dma接收完成中断处理
 * @param uart_id 串口ID
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
 * @brief  串口dma接收缓存大小一半数据中断处理
 * @param uart_id 串口ID
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
 * @brief  串口空闲中断处理
 * @param uart_id 串口ID
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
 * @brief  串口dma发送完成中断处理
 * @param uart_id 串口ID
 * @retval void
 */
void uart_dmatx_done_isr(uint8_t uart_id)
{
    s_uart_dev[uart_id].status = 0;	/* DMA发送空闲 */
}

/**
 * @brief  循环从串口发送fifo读出数据,放置于dma发送缓存,并启动dma传输
 * @param uart_id 串口ID
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

        s_uart_dev[uart_id].status = 0x01;	/* DMA发送状态 */
        bsp_uart_dmatx_config(uart_id, s_uart_dev[uart_id].dmatx_buf, size);
    }
}

/**
 * @brief  设置串口波特率
 * @param  uart_id: 串口ID (DEV_UART1, DEV_UART3等)
 * @param  baudrate: 要设置的波特率值
 * @retval UART_SET_BAUDRATE_OK: 成功
 *         UART_SET_BAUDRATE_ERROR: 失败
 */
uint8_t uart_set_baudrate(uint8_t uart_id, uint32_t baudrate)
{
    UART_HandleTypeDef *huart = NULL;
    
    // 获取对应串口句柄
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
            return UART_SET_BAUDRATE_ERROR;  // 无效的串口ID
    }
    
    // 停止当前的DMA传输
    HAL_UART_DMAStop(huart);
    
    // 更新波特率
    huart->Init.BaudRate = baudrate;
    if (HAL_UART_Init(huart) != HAL_OK)
    {
        return UART_SET_BAUDRATE_ERROR;
    }
    
    // 重新配置DMA接收
    uart_device_init(uart_id);
    
    return UART_SET_BAUDRATE_OK;
}

/**
 * @brief  清空串口缓冲区
 * @param  uart_id: 串口ID (DEV_UART1, DEV_UART3等)
 * @param  config: 清空配置参数,NULL则使用默认配置
 * @retval uart_flush_status_t: 清空状态
 */
uart_flush_status_t uart_flush_buffer(uint8_t uart_id, const uart_flush_config_t *config)
{
    // 默认配置
    static const uart_flush_config_t default_config = {
        .timeout_ms = 1000,  // 默认1秒超时
        .chunk_size = 64,    // 默认每次读取64字节
        .delay_ms = 1        // 默认1ms延时
    };
    
    // 使用传入的配置或默认配置
    const uart_flush_config_t *cfg = config ? config : &default_config;
    
    // 使用栈上分配的固定大小缓冲区,避免动态内存分配
    uint8_t temp_buf[64];
    uint16_t read_size = cfg->chunk_size > sizeof(temp_buf) ? 
                        sizeof(temp_buf) : cfg->chunk_size;
    
    uint32_t start_time = HAL_GetTick();
    uint32_t total_bytes = 0;
    
    while(1) {
        // 检查超时
        if(HAL_GetTick() - start_time > cfg->timeout_ms) {
            return UART_FLUSH_TIMEOUT;
        }
        
        // 读取数据
        uint16_t len = uart_read(uart_id, temp_buf, read_size);
        if(len > 0) {
            total_bytes += len;
            HAL_Delay(cfg->delay_ms);
        } else {
            // 没有更多数据,清空完成
            break;
        }
    }
    
    return UART_FLUSH_OK;
}

/**
 * @brief  使用默认参数清空串口缓冲区
 * @param  uart_id: 串口ID (DEV_UART1, DEV_UART3等)
 * @retval uart_flush_status_t: 清空状态
 */
uart_flush_status_t uart_flush(uint8_t uart_id)
{
    return uart_flush_buffer(uart_id, NULL);
}

/**
 * @brief  串口1直接输出(不格式化)函数
 * @param  str: 要输出的字符串
 * @param  len: 要输出的字符串长度
 * @retval 实际发送的字节数
 */
int uart1_print(const char *str, uint16_t len)
{
    int ret;
    
    // 直接使用uart_write发送数据
    ret = uart_write(DEV_UART1, (uint8_t*)str, len);
    
    // 启动DMA传输
    uart_poll_dma_tx(DEV_UART1);
    
    return ret;
}

/**
 * @brief  串口1格式化输出函数
 * @param  format: 格式化字符串
 * @param  ...: 可变参数
 * @retval 实际发送的字节数
 */
int uart1_printf(const char *format, ...)
{
    va_list args;
    uint8_t buf[512];  // 增大缓冲区以支持更长的数据
    int length;
    uint16_t ret;

    // 格式化字符串
    va_start(args, format);
    length = vsnprintf((char *)buf, sizeof(buf), format, args);
    va_end(args);

    // 检查缓冲区是否溢出
    if (length < 0 || length >= sizeof(buf)) {
        return 0;
    }

    // 发送格式化后的数据
    ret = uart_write(DEV_UART1, buf, length);
    
    // 激活DMA传输
    uart_poll_dma_tx(DEV_UART1);
    
    return ret;
}

/**
 * @brief  串口3格式化输出函数
 * @param  format: 格式化字符串
 * @param  ...: 可变参数
 * @retval 实际发送的字节数
 */
int uart3_printf(const char *format, ...)
{
    va_list args;
    uint8_t buf[512];  // 增大缓冲区以支持更长的数据
    int length;
    uint16_t ret;

    // 格式化字符串
    va_start(args, format);
    length = vsnprintf((char *)buf, sizeof(buf), format, args);
    va_end(args);

    // 检查缓冲区是否溢出
    if (length < 0 || length >= sizeof(buf)) {
        return 0;
    }

    // 发送格式化后的数据
    ret = uart_write(DEV_UART3, buf, length);
    
    // 激活DMA传输
    uart_poll_dma_tx(DEV_UART3);
    
    return ret;
}

/**
 * @brief  串口4格式化输出函数
 * @param  format: 格式化字符串
 * @param  ...: 可变参数
 * @retval 实际发送的字节数
 */
int uart4_printf(const char *format, ...)
{
    va_list args;
    uint8_t buf[512];  // 增大缓冲区以支持更长的数据
    int length;
    uint16_t ret;

    // 格式化字符串
    va_start(args, format);
    length = vsnprintf((char *)buf, sizeof(buf), format, args);
    va_end(args);

    // 检查缓冲区是否溢出
    if (length < 0 || length >= sizeof(buf)) {
        return 0;
    }

    // 发送格式化后的数据
    ret = uart_write(DEV_UART4, buf, length);
    
    // 激活DMA传输
    uart_poll_dma_tx(DEV_UART4);
    
    return ret;
}

/**
 * @brief  串口5格式化输出函数
 * @param  format: 格式化字符串
 * @param  ...: 可变参数
 * @retval 实际发送的字节数
 */
int uart5_printf(const char *format, ...)
{
    va_list args;
    uint8_t buf[512];  // 增大缓冲区以支持更长的数据
    int length;
    uint16_t ret;

    // 格式化字符串
    va_start(args, format);
    length = vsnprintf((char *)buf, sizeof(buf), format, args);
    va_end(args);

    // 检查缓冲区是否溢出
    if (length < 0 || length >= sizeof(buf)) {
        return 0;
    }

    // 发送格式化后的数据
    ret = uart_write(DEV_UART5, buf, length);
    
    // 激活DMA传输
    uart_poll_dma_tx(DEV_UART5);
    
    return ret;
}
