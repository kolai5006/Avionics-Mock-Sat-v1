#pragma once

#include "stm32l4xx_hal.h"

#include <string.h>

/* Configuration */
#define HC05_RX_BUFFER_SIZE  256
#define HC05_TX_BUFFER_SIZE  256

/* HC-05 Driver error */
typedef enum{
	HC05_SUCCESS        = 0U,
	HC05_NOT_DETECTED   = 1U,
	HC05_BUSY			= 2U,
	HC05_NOT_BUSY		= 4U,
} HC05_ERROR;


/* HC-05 Driver Structure */
typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_rx;
    DMA_HandleTypeDef  *hdma_tx;

    /* RX Circular Buffer */
    uint8_t rx_buffer[HC05_RX_BUFFER_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;

    /* TX Buffer */
    uint8_t tx_buffer[HC05_TX_BUFFER_SIZE];
    HC05_ERROR tx_busy;

    /* Status */
    volatile HC05_ERROR initialized;
} HC05_HandleTypeDef;

HC05_ERROR	HC05_Init(UART_HandleTypeDef *huart);
HC05_ERROR 	HC05_Transmit(uint8_t *data, uint16_t len);
HC05_ERROR  HC05_TransmitString(const char *str);
uint16_t 	HC05_Available(void);
HC05_ERROR 	HC05_Read(uint8_t *data);
uint16_t 	HC05_ReadBytes(uint8_t *buffer, uint16_t len);
uint16_t 	HC05_ReadLine(char *buffer, uint16_t max_len);
HC05_ERROR	HC05_IsTxComplete(void);

void 		HC05_FlushRx(void);


