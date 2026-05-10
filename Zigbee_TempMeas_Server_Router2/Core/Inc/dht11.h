/**
  ******************************************************************************
  * @file    dht11.h
  * @brief   DHT11 Temperature and Humidity Sensor Driver
  *          Supports Waveshare DHT11 module
  ******************************************************************************
  */

#ifndef __DHT11_H
#define __DHT11_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbaxx_hal.h"
#include "stm32wbaxx_ll_gpio.h"

/* Defines -------------------------------------------------------------------*/
#define DHT11_PIN                 LL_GPIO_PIN_15          /* PA15 */
#define DHT11_PORT                GPIOA
#define DHT11_TIMEOUT             100u                    /* ms */

/* DHT11 Sensor States */
typedef enum {
  DHT11_OK = 0u,
  DHT11_ERROR_TIMEOUT = 1u,
  DHT11_ERROR_CHECKSUM = 2u,
  DHT11_ERROR_NO_RESPONSE = 3u,
  DHT11_ERROR_NOT_INITIALIZED = 4u
} DHT11_StatusTypeDef;

/* DHT11 Data Structure */
typedef struct {
  uint8_t  humidity_int;        /* Integer part of humidity (0-100) */
  uint8_t  humidity_dec;        /* Decimal part of humidity (0-99) */
  uint8_t  temperature_int;     /* Integer part of temperature */
  uint8_t  temperature_dec;     /* Decimal part of temperature */
  uint8_t  checksum;            /* Checksum */
} DHT11_DataTypeDef;

/* Function Prototypes -------------------------------------------------------*/

/**
  * @brief  Initialize DHT11 sensor GPIO
  * @retval DHT11_StatusTypeDef - Status of initialization
  */
DHT11_StatusTypeDef DHT11_Init(void);

/**
  * @brief  Read temperature and humidity from DHT11 sensor
  * @param  data - Pointer to DHT11_DataTypeDef structure
  * @retval DHT11_StatusTypeDef - Status of read operation
  */
DHT11_StatusTypeDef DHT11_Read(DHT11_DataTypeDef *data);

/**
  * @brief  Get temperature in centigrade (centidegrees for Zigbee)
  * @param  data - Pointer to DHT11_DataTypeDef structure
  * @retval int16_t - Temperature in centidegrees (e.g., 2500 for 25.00°C)
  */
int16_t DHT11_GetTemperatureCC(const DHT11_DataTypeDef *data);

/**
  * @brief  Get humidity in percentage
  * @param  data - Pointer to DHT11_DataTypeDef structure
  * @retval uint16_t - Humidity in percentage (e.g., 6500 for 65.00%)
  */
uint16_t DHT11_GetHumidityCC(const DHT11_DataTypeDef *data);

/**
  * @brief  Check if sensor is available (performs a read test)
  * @retval 1 if sensor responds, 0 if no sensor or error
  */
uint8_t DHT11_IsAvailable(void);

#ifdef __cplusplus
}
#endif

#endif /* __DHT11_H */
