/**
  ******************************************************************************
  * @file    dht11.c
  * @brief   DHT11 Temperature and Humidity Sensor Driver Implementation
  ******************************************************************************
  */

#include "dht11.h"
#include "stm32wbaxx_ll_gpio.h"
#include "stm32wbaxx_ll_bus.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define DHT11_START_LOW_DURATION    18000u   /* 18ms in microseconds */
#define DHT11_RESPONSE_TIMEOUT      100000u  /* 100ms timeout */
#define DHT11_BIT_READ_TIMEOUT      100u     /* 100µs timeout per bit */
#define DHT11_INIT_WAIT             1000u    /* 1s wait after init */

static uint8_t dht11_initialized = 0u;

/* Private function prototypes -----------------------------------------------*/
static void DHT11_GPIO_SetInput(void);
static void DHT11_GPIO_SetOutput(void);
static uint8_t DHT11_GPIO_ReadPin(void);
static void DHT11_GPIO_SetLow(void);
static void DHT11_GPIO_SetHigh(void);
static void DHT11_DelayUs(uint32_t us);
static DHT11_StatusTypeDef DHT11_SendStartSignal(void);
static DHT11_StatusTypeDef DHT11_WaitResponse(void);
static DHT11_StatusTypeDef DHT11_ReadBits(uint8_t *data);
static uint8_t DHT11_VerifyChecksum(const uint8_t *data);

/* Private function implementations ------------------------------------------*/

/**
  * @brief  Set GPIO as input (floating/high impedance)
  */
static void DHT11_GPIO_SetInput(void)
{
  LL_GPIO_SetPinMode(DHT11_PORT, DHT11_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(DHT11_PORT, DHT11_PIN, LL_GPIO_PULL_UP);
}

/**
  * @brief  Set GPIO as output (open-drain)
  */
static void DHT11_GPIO_SetOutput(void)
{
  LL_GPIO_SetPinMode(DHT11_PORT, DHT11_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(DHT11_PORT, DHT11_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(DHT11_PORT, DHT11_PIN, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinSpeed(DHT11_PORT, DHT11_PIN, LL_GPIO_SPEED_FREQ_HIGH);
}

/**
  * @brief  Read GPIO pin state
  */
static uint8_t DHT11_GPIO_ReadPin(void)
{
  return (LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) ? 1u : 0u);
}

/**
  * @brief  Set GPIO to low
  */
static void DHT11_GPIO_SetLow(void)
{
  LL_GPIO_ResetOutputPin(DHT11_PORT, DHT11_PIN);
}

/**
  * @brief  Set GPIO to high (open-drain, releases the line)
  */
static void DHT11_GPIO_SetHigh(void)
{
  LL_GPIO_SetOutputPin(DHT11_PORT, DHT11_PIN);
}

/**
  * @brief  Send START signal to DHT11
  */
static DHT11_StatusTypeDef DHT11_SendStartSignal(void)
{
  /* Set as output */
  DHT11_GPIO_SetOutput();
  
  /* Pull down for 18ms */
  DHT11_GPIO_SetLow();
  DHT11_DelayUs(DHT11_START_LOW_DURATION);
  
  /* Release (pull high via pull-up resistor) */
  DHT11_GPIO_SetHigh();
  
  /* Wait 20-40µs */
  DHT11_DelayUs(30u);
  
  return DHT11_OK;
}

/**
  * @brief  Wait for DHT11 response
  */
static DHT11_StatusTypeDef DHT11_WaitResponse(void)
{
  uint32_t timeout;
  
  /* Set as input */
  DHT11_GPIO_SetInput();
  
  /* DHT11 should pull down for 80µs */
  timeout = 0u;
  while (DHT11_GPIO_ReadPin() == 1u && timeout < DHT11_RESPONSE_TIMEOUT)
  {
    timeout++;
    DHT11_DelayUs(1u);
  }
  
  if (timeout >= DHT11_RESPONSE_TIMEOUT)
  {
    return DHT11_ERROR_NO_RESPONSE;
  }
  
  /* Wait for DHT11 to release (go high) */
  timeout = 0u;
  while (DHT11_GPIO_ReadPin() == 0u && timeout < DHT11_RESPONSE_TIMEOUT)
  {
    timeout++;
    DHT11_DelayUs(1u);
  }
  
  if (timeout >= DHT11_RESPONSE_TIMEOUT)
  {
    return DHT11_ERROR_NO_RESPONSE;
  }
  
  return DHT11_OK;
}

/**
  * @brief  Read 40 bits of data from DHT11
  */
static DHT11_StatusTypeDef DHT11_ReadBits(uint8_t *data)
{
  uint32_t i, j;
  uint32_t timeout;
  uint8_t bit_value;
  
  /* Read 5 bytes (40 bits) */
  for (i = 0u; i < 5u; i++)
  {
    data[i] = 0u;
    
    for (j = 0u; j < 8u; j++)
    {
      /* Wait for DHT11 to pull low */
      timeout = 0u;
      while (DHT11_GPIO_ReadPin() == 1u && timeout < DHT11_BIT_READ_TIMEOUT)
      {
        timeout++;
        DHT11_DelayUs(1u);
      }
      
      if (timeout >= DHT11_BIT_READ_TIMEOUT)
      {
        return DHT11_ERROR_TIMEOUT;
      }
      
      /* Wait for DHT11 to release (go high) */
      timeout = 0u;
      while (DHT11_GPIO_ReadPin() == 0u && timeout < DHT11_BIT_READ_TIMEOUT)
      {
        timeout++;
        DHT11_DelayUs(1u);
      }
      
      if (timeout >= DHT11_BIT_READ_TIMEOUT)
      {
        return DHT11_ERROR_TIMEOUT;
      }
      
      /* Wait ~35µs and check the pin state */
      /* If pin is still high, it's a '1', otherwise '0' */
      DHT11_DelayUs(40u);
      bit_value = DHT11_GPIO_ReadPin();
      
      data[i] = (data[i] << 1u) | (bit_value ? 1u : 0u);
      
      /* Wait until pin goes low again */
      timeout = 0u;
      while (DHT11_GPIO_ReadPin() == 1u && timeout < DHT11_BIT_READ_TIMEOUT)
      {
        timeout++;
        DHT11_DelayUs(1u);
      }
    }
  }
  
  return DHT11_OK;
}

/**
  * @brief  Verify DHT11 checksum
  */
static uint8_t DHT11_VerifyChecksum(const uint8_t *data)
{
  uint8_t checksum;
  
  checksum = data[0] + data[1] + data[2] + data[3];
  
  return (checksum == data[4]) ? 1u : 0u;
}

/**
  * @brief  Microsecond-level delay using simple loop
  *         Note: Accuracy depends on CPU frequency and optimization level
  *         For STM32WBA55 at ~32MHz, approx 1µs per 8 NOPs
  */
static void DHT11_DelayUs(uint32_t us)
{
  /* Simple delay loop. With compiler optimization, each iteration ~32 clock cycles */
  /* At 32MHz: 32 cycles = 1µs */
  uint32_t i;
  for (i = 0u; i < us; i++)
  {
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
  }
}

/* Public function implementations -------------------------------------------*/

/**
  * @brief  Initialize DHT11 sensor
  */
DHT11_StatusTypeDef DHT11_Init(void)
{
  /* Enable GPIO clock */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  
  /* Configure GPIO for DHT11 as input initially */
  LL_GPIO_SetPinMode(DHT11_PORT, DHT11_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinOutputType(DHT11_PORT, DHT11_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(DHT11_PORT, DHT11_PIN, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinSpeed(DHT11_PORT, DHT11_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  
  /* Set pin high initially */
  DHT11_GPIO_SetHigh();
  
  /* Wait for sensor to stabilize */
  DHT11_DelayUs(DHT11_INIT_WAIT);
  
  dht11_initialized = 1u;
  
  return DHT11_OK;
}

/**
  * @brief  Read temperature and humidity from DHT11
  */
DHT11_StatusTypeDef DHT11_Read(DHT11_DataTypeDef *data)
{
  uint8_t raw_data[5];
  DHT11_StatusTypeDef status;
  
  if (data == NULL || !dht11_initialized)
  {
    return DHT11_ERROR_NOT_INITIALIZED;
  }
  
  /* Send START signal */
  status = DHT11_SendStartSignal();
  if (status != DHT11_OK)
  {
    return status;
  }
  
  /* Wait for response */
  status = DHT11_WaitResponse();
  if (status != DHT11_OK)
  {
    return status;
  }
  
  /* Read 40 bits of data */
  status = DHT11_ReadBits(raw_data);
  if (status != DHT11_OK)
  {
    return status;
  }
  
  /* Verify checksum */
  if (!DHT11_VerifyChecksum(raw_data))
  {
    return DHT11_ERROR_CHECKSUM;
  }
  
  /* Parse data */
  data->humidity_int = raw_data[0];
  data->humidity_dec = raw_data[1];
  data->temperature_int = raw_data[2];
  data->temperature_dec = raw_data[3];
  data->checksum = raw_data[4];
  
  return DHT11_OK;
}

/**
  * @brief  Get temperature in centidegrees (for Zigbee)
  */
int16_t DHT11_GetTemperatureCC(const DHT11_DataTypeDef *data)
{
  if (data == NULL)
  {
    return 0;
  }
  
  return (int16_t)((data->temperature_int * 100) + data->temperature_dec);
}

/**
  * @brief  Get humidity in centipercent (centipercentage)
  */
uint16_t DHT11_GetHumidityCC(const DHT11_DataTypeDef *data)
{
  if (data == NULL)
  {
    return 0u;
  }
  
  return (uint16_t)((data->humidity_int * 100u) + data->humidity_dec);
}

/**
  * @brief  Check if DHT11 sensor is available
  */
uint8_t DHT11_IsAvailable(void)
{
  DHT11_DataTypeDef data;
  
  if (!dht11_initialized)
  {
    return 0u;
  }
  
  return (DHT11_Read(&data) == DHT11_OK) ? 1u : 0u;
}
