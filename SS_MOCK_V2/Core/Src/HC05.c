/*
 * HC05.c
 *
 *  Created on: Oct 17, 2025
 *      Author: brasi
 */
#include "HC05.h"


HC05_HandleTypeDef hc05;
/**
 * @brief Initialize HC-05 driver with DMA
 */
HC05_ERROR HC05_Init(UART_HandleTypeDef *huart){

	if (huart == NULL) {
	        return HC05_NOT_DETECTED;
	    }

	    /* Initialize structure */
	    hc05.huart = huart;
	    hc05.hdma_rx = huart->hdmarx;
	    hc05.hdma_tx = huart->hdmatx;
	    hc05.rx_head = 0;
	    hc05.rx_tail = 0;
	    hc05.tx_busy = HC05_NOT_BUSY;
	    hc05.initialized = HC05_NOT_DETECTED;

	    /* Clear buffers */
	    memset(hc05.rx_buffer, 0, HC05_RX_BUFFER_SIZE);
	    memset(hc05.tx_buffer, 0, HC05_TX_BUFFER_SIZE);

	    /* Start DMA reception in circular mode */
	    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart,
	                                                             hc05.rx_buffer,
	                                                             HC05_RX_BUFFER_SIZE);

	    if (status == HAL_OK) {
	        /* Disable half transfer interrupt (we only need full/idle) */
	        __HAL_DMA_DISABLE_IT(hc05.hdma_rx, DMA_IT_HT);
	        hc05.initialized = HC05_SUCCESS;
	    }

	    return HC05_SUCCESS;

}

/**
 * @brief Send data via HC-05 using DMA
 */
HC05_ERROR 	HC05_Transmit(uint8_t *data, uint16_t len){

    if (hc05.initialized == HC05_NOT_DETECTED || data == NULL || len == 0) {
        return HC05_NOT_DETECTED;
    }

    /* Check if previous transmission is complete */
    if (hc05.tx_busy == HC05_BUSY) {
        return HC05_BUSY;
    }

    /* Limit length to buffer size */
    if (len > HC05_TX_BUFFER_SIZE) {
        len = HC05_TX_BUFFER_SIZE;
    }

    /* Copy data to TX buffer */
    memcpy(hc05.tx_buffer, data, len);
    hc05.tx_busy = HC05_BUSY;

    /* Start DMA transmission */

    return HAL_UART_Transmit_DMA(hc05.huart, hc05.tx_buffer, len);
}

/**
 * @brief Send string via HC-05
 */
HC05_ERROR  HC05_TransmitString(const char *str){

    if (str == NULL) {
        return HC05_NOT_DETECTED;
    }

    uint16_t len = strlen(str);
    return HC05_Transmit((uint8_t*)str, len);
}

/**
 * @brief Get number of bytes available in RX buffer
 */
uint16_t 	HC05_Available(void){

    if (hc05.initialized != HC05_SUCCESS) {
        return 0;
    }

    /* Calculate current DMA write position */
    uint16_t dma_pos = HC05_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hc05.hdma_rx);

    /* Update head position */
    hc05.rx_head = dma_pos;

    /* Calculate available bytes */
    if (hc05.rx_head >= hc05.rx_tail) {
        return hc05.rx_head - hc05.rx_tail;
    } else {
        return HC05_RX_BUFFER_SIZE - hc05.rx_tail + hc05.rx_head;
    }

}

/**
 * @brief Read one byte from RX buffer
 */
HC05_ERROR 	HC05_Read(uint8_t *data)
{
    if (HC05_Available() == 0) {
        return HC05_NOT_DETECTED;
    }

    *data = hc05.rx_buffer[hc05.rx_tail];
    hc05.rx_tail = (hc05.rx_tail + 1) % HC05_RX_BUFFER_SIZE;

    return HC05_SUCCESS;
}

/**
 * @brief Read multiple bytes from RX buffer
 */
uint16_t HC05_ReadBytes(uint8_t *buffer, uint16_t len)
{
    if (buffer == NULL || len == 0) {
        return 0;
    }

    uint16_t available = HC05_Available();
    uint16_t to_read = (len < available) ? len : available;
    uint16_t bytes_read = 0;

    while (bytes_read < to_read) {
        buffer[bytes_read] = hc05.rx_buffer[hc05.rx_tail];
        hc05.rx_tail = (hc05.rx_tail + 1) % HC05_RX_BUFFER_SIZE;
        bytes_read++;
    }

    return bytes_read;
}

/**
 * @brief Read line until newline character
 */
uint16_t HC05_ReadLine(char *buffer, uint16_t max_len)
{
    if (buffer == NULL || max_len == 0) {
        return 0;
    }

    uint16_t count = 0;
    uint8_t byte;

    while (count < (max_len - 1)) {
        if (HC05_Read(&byte) == HC05_NOT_DETECTED) {
            break;
        }

        buffer[count++] = (char)byte;

        /* Check for newline */
        if (byte == '\n' || byte == '\r') {
            break;
        }
    }

    buffer[count] = '\0';
    return count;
}

/**
 * @brief Check if transmission is complete
 */
HC05_ERROR HC05_IsTxComplete(void)
{
	if (hc05.tx_busy == HC05_NOT_BUSY){
		return HC05_SUCCESS;
	}
	else return HC05_BUSY;


}

/**
 * @brief Clear RX buffer
 */
void HC05_FlushRx(void)
{
    uint16_t dma_pos = HC05_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hc05.hdma_rx);
    hc05.rx_tail = dma_pos;
    hc05.rx_head = dma_pos;
}

/**
 * @brief UART RX Event Callback
 * Call this from HAL_UARTEx_RxEventCallback in your main.c
 */
void HC05_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == hc05.huart) {
        /* Update head position */
        hc05.rx_head = Size;

        /* Restart DMA reception */
        HAL_UARTEx_ReceiveToIdle_DMA(hc05.huart, hc05.rx_buffer, HC05_RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(hc05.hdma_rx, DMA_IT_HT);
    }
}

/**
 * @brief UART TX Complete Callback
 * Call this from HAL_UART_TxCpltCallback in your main.c
 */
void HC05_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == hc05.huart) {
        hc05.tx_busy = HC05_NOT_BUSY;
    }
}



