/*****************************************************************************
* | File      :   EPD_Display_Values.h
* | Author      :   Modified for temperature and RGB display
* | Function    :   Display temperature and RGB values on e-Paper
*
******************************************************************************/
#ifndef _EPD_DISPLAY_VALUES_H_
#define _EPD_DISPLAY_VALUES_H_

#include <stdint.h>

/**
 * Clear the e-Paper display
 */
void EPD_Display_Clear(void);

/**
 * Display RGB value on left side (xxx.x format)
 * @param value: RGB value 0-255, displayed as 0-25.5 (divided by 10)
 * @param color: 0=Red, 1=Green, 2=Blue
 */
void EPD_Display_RGB(uint8_t value, uint8_t color);

/**
 * Display temperature on right side (xx.x format)
 * @param temp: Temperature value (e.g., 21.2 for 21.2°C)
 */
void EPD_Display_Temperature(float temp);

/**
 * Display Temperature and RGB values sequentially
 * @param temp: Temperature value
 * @param r: Red value 0-255
 * @param g: Green value 0-255
 * @param b: Blue value 0-255
 */
void EPD_Display_Sequence(float temp, uint8_t r, uint8_t g, uint8_t b);

/**
 * Display startup sequence (0-9) on all used digits
 */
void EPD_Display_Startup_Sequence(void);

#endif
