/**
 ******************************************************************************
 * @file    TCA9546A.c
 * @brief   TCA9546A I2C Switch Driver Implementation
 ******************************************************************************
 *
 * will finnish this documentation section later
 *
 *
 ******************************************************************************
 */

#include "tca9546a.h"

/* PRIVATE DEFINES */
/* Constants used internally by the driver */

/* TCA9546A base I2C address
 * The actual address is 0x70 to 0x77 depending on A0, A1, A2 pins */
#define TCA9546A_BASE_ADDR 0x70U

/* How long to wait for I2C operations (milliseconds)
 * If I2C takes longer than this, we give up and return an error
 * Can remove later if you want me to or add more time */
#define TCA9546A_WRITE_TIMEOUT 100U /* 100ms to send data */
#define TCA9546A_READ_TIMEOUT 100U  /* 100ms to receive data */

/* GLOBAL VARIABLES */

/* Pointer to the I2C peripheral we're using
 * This gets set when you call TCA9546A_I2C_Mount()
 * (basically a phone line to the mux :p) */
I2C_HandleTypeDef *TCA9546A_I2C;

/* State Structure
 * (driver's "memory") */
typedef struct
{
    uint8_t device_address;   /* I2C address of the TCA9546A (0x70-0x77) */
    uint8_t current_channels; /* Which channels are currently open (4 channels, bits 0-3) */
} TCA9546A_STATE;

/* Create one instance of the state structure */
TCA9546A_STATE TCA9546A_state;

#ifdef TCA9546A_HARDWARE_RESET
/* Optional: GPIO pin for hardware reset
 * uncommented TCA9546A_HARDWARE_RESET in the .h file if you want to use this!!!! */
GPIO_TypeDef *Reset_Port = NULL; // Which GPIO port (GPIOA, GPIOB, etc.)
uint16_t Reset_Pin = 0;          // Which pin number
#endif

/* PRIVATE FUNCTION PROTOTYPES */
/* These functions are only used inside this file */

static inline HAL_StatusTypeDef TCA9546A_Write(uint8_t data);
static inline HAL_StatusTypeDef TCA9546A_Read(uint8_t *buf);
static inline void TCA9546A_ERROR_HANDLE(HAL_StatusTypeDef status);

/* PRIVATE FUNCTION IMPLEMENTATIONS */

/**
 * @brief Write one byte to the TCA9546A
 *
 * HOW IT WORKS:
 *   This function sends a single byte over I2C to the TCA9546A.
 *   That byte controls which channels are open/closed.
 *
 * THE BYTE:
 *   Each bit in the byte represents a channel:
 *   Bit 0 = Channel 0, Bit 1 = Channel 1, ...
 *   If bit is 1 = channel is ON
 *   If bit is 0 = channel is OFF
 *
 * WHAT HAPPENS:
 *   1. HAL_I2C_Master_Transmit sends the byte over I2C
 *   2. TCA9546A receives it and flips its internal switches
 *   3. Function returns status if it worked or not
 *
 * @param data The byte to send (controls which channels to open)
 * @return HAL_OK if successful, HAL_ERROR/HAL_TIMEOUT if failed
 */
static inline HAL_StatusTypeDef TCA9546A_Write(uint8_t data)
{

    HAL_StatusTypeDef status; /* Will store the result: HAL_OK or HAL_ERROR */

    /* will fill in later but if confused lmk */
    status = HAL_I2C_Master_Transmit(TCA9546A_I2C,
                                     TCA9546A_state.device_address << 1,
                                     &data,
                                     1,
                                     TCA9546A_WRITE_TIMEOUT);

    return status;
}

/**
 * @brief Read one byte from the TCA9546A
 *
 * HOW IT WORKS:
 *   Will see which channels are avaibklable
 *   and stores the answer in the buffer you'll provide.
 *
 * THE BYTE YOU GET:
 *   Each bit tells you if a channel is open:
 *   Example: 0b00001001 means channels 0 and 3 are open
 *
 * @param buf Pointer to where the received byte should be stored
 * @return HAL_OK if successful, HAL_ERROR/HAL_TIMEOUT if failed
 */
static inline HAL_StatusTypeDef TCA9546A_Read(uint8_t *buf)
{

    HAL_StatusTypeDef status;

    /* will fill */
    status = HAL_I2C_Master_Receive(TCA9546A_I2C,
                                    TCA9546A_state.device_address << 1,
                                    buf,
                                    1,
                                    TCA9546A_READ_TIMEOUT);

    return status;
}

/**
 * @brief Check if there was an error and stop if so
 *
 * WHAT IT DOES:
 *   Checks if the status is HAL_OK (the pass status :D ).
 *   If not, freezes the program in an infinite loop.
 *
 * WHY THO?
 *   Cause it's better to stop than continue with bad data >:(
 *
 * NOTE FOR LATER ON:
 *   You might wanna replace the while(1) with something else
 *   With what? idk yet idk what's the best option atm
 *
 * @param status The status code to check (from HAL function)
 */
static inline void TCA9546A_ERROR_HANDLE(HAL_StatusTypeDef status)
{

    /* If status is NOT HAL_OK, something went wrong */
    if (status != HAL_OK)
    {

        /* Turn on an error LED so you know there's a problem
         * You'd have to uncomment the line below if you have an LED connected */
        // HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_SET);

        /* Freeze the program */
        while (1)
        {
            /* IF STUCK: Check: (chatgpt generated so if none works blame chat)
             *      Is TCA9546A powered?
             *      Are SDA and SCL connected?
             *      Are pull-up resistors present?
             *      Is the I2C address correct?
             *      Is I2C peripheral enabled in STM32?
             */
        }
    }

    /* If we get here, status was HAL_OK basically passed */
}

/* PUBLIC FUNCTION IMPLEMENTATIONS */
/* These are the functions you call from your main.c */

/**
 * @brief Tell the driver which I2C peripheral to use
 *
 * WHAT IT DOES:
 *   Saves a pointer to your I2C handle so all the other functions
 *   know which I2C bus to use when talking to the TCA9546A.
 *
 * NOTE: Call this first before calling any other TCA9546A functions
 *
 * SO FOR EXAMPLE:
 *   TCA9546A_I2C_Mount(&hi2c1);  // Use I2C1
 */
void TCA9546A_I2C_Mount(I2C_HandleTypeDef *i2c)
{
    /* Store the pointer to the I2C handle
     * Now all other functions can use "TCA9546A_I2C" to talk to the device */
    TCA9546A_I2C = i2c;
}

#ifdef TCA9546A_HARDWARE_RESET
/**
 * @brief Tell the driver which GPIO pin is connected to RESET

 * EXAMPLE:
 *   TCA9546A_HW_Reset_Mount(GPIOA, GPIO_PIN_8);  // Reset on PA8
 */
void TCA9546A_HW_Reset_Mount(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin)
{
    /* Save which port and pin to use */
    Reset_Port = GPIO_Port;
    Reset_Pin = GPIO_Pin;
}
#endif

/**
 * @brief Initialize the TCA9546A and prepare it for use
 */
TCA9546A_ERROR TCA9546A_Init(uint8_t a2, uint8_t a1, uint8_t a0)
{

    HAL_StatusTypeDef status;

    /* Can show Calulation explanation later if needed */
    TCA9546A_state.device_address = TCA9546A_BASE_ADDR |
                                    ((a2 & 0x01) << 2) |
                                    ((a1 & 0x01) << 1) |
                                    (a0 & 0x01);

    HAL_Delay(10);

    /* Check if device is present on the I2C bus
     *
     * HAL_OK: Device responded
     * HAL_ERROR/HAL_TIMEOUT: No response
     */
    status = HAL_I2C_IsDeviceReady(TCA9546A_I2C,
                                   TCA9546A_state.device_address << 1,
                                   3,
                                   TCA9546A_WRITE_TIMEOUT);

    /* If device didn't respond, return error */
    if (status != HAL_OK)
    {
        return TCA9546A_NOT_DETECTED; /* Gotta check ur wiring */
    }

#ifdef TCA9546A_HARDWARE_RESET
    /* Optional hardware reset on/off */
    if (Reset_Port != NULL)
    {                                                             /* Only if reset pin was configured */
        HAL_GPIO_WritePin(Reset_Port, Reset_Pin, GPIO_PIN_RESET); /* Pull LOW */
        HAL_Delay(1);                                             /* Wait 1ms */
        HAL_GPIO_WritePin(Reset_Port, Reset_Pin, GPIO_PIN_SET);   /* Pull HIGH */
        HAL_Delay(10);                                            /* Wait 10ms */
    }
#endif

    /* Turn off all channels (clean starting point) */
    status = TCA9546A_Write(TCA9546A_NO_CHANNEL); /* Send 0x00 */
    TCA9546A_ERROR_HANDLE(status);                /* Stop if it failed */

    /* channels are now closed */
    TCA9546A_state.current_channels = TCA9546A_NO_CHANNEL;

    /* ready to use again */
    return TCA9546A_SUCCESS;
}

/**
 * @brief Select which channels should be open
 *
 * @param channels Byte where each bit controls one channel
 * @return TCA9546A_SUCCESS if successful
 */
TCA9546A_ERROR TCA9546A_Select_Channels(uint8_t channels)
{

    HAL_StatusTypeDef status;

    /* Send the channel selection byte to TCA9546A over I2C */
    status = TCA9546A_Write(channels);
    TCA9546A_ERROR_HANDLE(status); /* Stop if it failed */

    /* Remember which channels we just opened*/
    TCA9546A_state.current_channels = channels;

    HAL_Delay(1);

    return TCA9546A_SUCCESS;
}

/**
 * @brief Enable a single channel (closes all others)
 *
 * @param channel Which channel to enable (0-3)
 * @return TCA9546A_SUCCESS if successful
 */
TCA9546A_ERROR TCA9546A_Enable_Channel(TCA9546A_CHANNEL channel)
{
    /* Just call Select_Channels with this one channel */
    return TCA9546A_Select_Channels((uint8_t)channel);
}

/**
 * @brief Close all channels
 *
 * @return TCA9546A_SUCCESS if successful
 */
TCA9546A_ERROR TCA9546A_Disable_All_Channels(void)
{
    /* Send 0x00 (all bits = 0) to close all channels */
    return TCA9546A_Select_Channels(TCA9546A_NO_CHANNEL);
}

/**
 * @brief Add channel(s) to the current selection
 *
 * @param channels Channel(s) to add
 * @return TCA9546A_SUCCESS if successful
 */
TCA9546A_ERROR TCA9546A_Add_Channels(uint8_t channels)
{
    /* bitwise OR to combine current channels with new ones */
    uint8_t new_channels = TCA9546A_state.current_channels | channels;

    return TCA9546A_Select_Channels(new_channels);
}

/**
 * @brief Remove channel(s) from the current selection
 *
 * @param channels Channel(s) to remove
 * @return TCA9546A_SUCCESS if successful
 */
TCA9546A_ERROR TCA9546A_Remove_Channels(uint8_t channels)
{
    /* bitwise AND with NOT to remove specific channels */
    uint8_t new_channels = TCA9546A_state.current_channels & ~channels;

    return TCA9546A_Select_Channels(new_channels);
}

/**
 * @brief Read which channels are currently active
 *
 * @param channels Pointer to variable where result will be stored
 * @return TCA9546A_SUCCESS if read was successful
 */
TCA9546A_ERROR TCA9546A_Get_Channels(uint8_t *channels)
{

    HAL_StatusTypeDef status;

    /* Read 1 byte from the TCA9546A, will byte tells us which channels are open */
    status = TCA9546A_Read(channels);
    TCA9546A_ERROR_HANDLE(status); /* Stop if it failed */

    /* Update our internal memory with what we just read */
    TCA9546A_state.current_channels = *channels;

    return TCA9546A_SUCCESS;
}

/**
 * @brief Check if the TCA9546A is present and responding
 *
 * NOTE: im blanking i return later
 *
 * RETURNS:
 *   true = yay it works and responded
 *   false = no response prob gotta check ur wiring
 *
 * @return true if device is present and responding
 *         false if no response (device not connected or wrong address)
 */
bool TCA9546A_Is_Present(void)
{

    HAL_StatusTypeDef status;

    /* If device is there, it will respond with "ACK" (acknowledge). */
    status = HAL_I2C_IsDeviceReady(TCA9546A_I2C,
                                   TCA9546A_state.device_address << 1,
                                   3,
                                   TCA9546A_WRITE_TIMEOUT);

    /* Convert HAL status to simple true/false
     * HAL_OK (0) = true, anything else = false */
    return (status == HAL_OK);
}