/*****************************************************************************
* | File      :   EPD_Display_Values.c
* | Author      :   Modified for temperature and RGB display
* | Function    :   Display temperature and RGB values on e-Paper
*
******************************************************************************/
#include "EPD_Display_Values.h"
#include "EPD_1in9.h"
#include <string.h>
#include <sys/_types.h>
#include "stm32wbaxx_hal.h"
#include "log_module.h" // Přidáno pro LOG_INFO_APP

// Buffer for display data
static unsigned char display_buffer[15];

// Helper function to get digit pattern
static const unsigned char* get_digit_pattern(uint8_t digit)
{
    switch(digit)
    {
        case 0: return DSPNUM_1in9_W0;
        case 1: return DSPNUM_1in9_W1;
        case 2: return DSPNUM_1in9_W2;
        case 3: return DSPNUM_1in9_W3; // Pro zobrazení '3', použij vzor z W4
        case 4: return DSPNUM_1in9_W4; // Pro zobrazení '4', použij vzor z W5
        case 5: return DSPNUM_1in9_W5; // Pro zobrazení '5', použij vzor z W6
        case 6: return DSPNUM_1in9_W6; // Pro zobrazení '6', použij vzor z W3
        case 7: return DSPNUM_1in9_W7;
        case 8: return DSPNUM_1in9_W8;
        case 9: return DSPNUM_1in9_W9;
        default: return DSPNUM_1in9_off;
    }
}

static void EPD_Update_Display(unsigned char *target_buffer)
{
    // Set DU_WB waveform and display directly - no cleanup to avoid darkening
    EPD_1in9_lut_DU_WB();
    EPD_1in9_Write_Screen(target_buffer);
    LOG_INFO_APP("E-paper: Display updated.");
}

/**
 * Clear the e-Paper display (all segments off) - full refresh sequence
 */
void EPD_Display_Clear(void)
{
    EPD_1in9_lut_5S();
    memcpy(display_buffer, DSPNUM_1in9_off, 15);
    EPD_1in9_Write_Screen(display_buffer);
}

/**
 * Display RGB value (xxx.x format)
 */
void EPD_Display_RGB(uint8_t value, uint8_t color)
{
    // Not implemented - use EPD_Display_Temperature instead
}

/**
 * Display temperature (xx.x format)
 */
void EPD_Display_Temperature(float temp)
{
    unsigned char target_buffer[15];
    memset(target_buffer, 0, 15);
    
    // Convert temperature to digits
    int temp_int = (int)(temp >= 0 ? temp * 10.0f + 0.5f : temp * 10.0f - 0.5f);
    if (temp_int < 0) temp_int = -temp_int;
    uint8_t temp_tens = (temp_int / 100) % 10;
    uint8_t temp_ones = (temp_int / 10) % 10;
    uint8_t temp_tenths = temp_int % 10;
    
    // Display temperature on bottom row (D4, D5, D6)
    if(temp_tens > 0) {
        target_buffer[5] = get_digit_pattern(temp_tens)[5];
        target_buffer[6] = get_digit_pattern(temp_tens)[6];
    }
    target_buffer[7] = get_digit_pattern(temp_ones)[7];
    target_buffer[8] = get_digit_pattern(temp_ones)[8];
    target_buffer[9] = get_digit_pattern(temp_tenths)[9];
    target_buffer[10] = get_digit_pattern(temp_tenths)[10];
    
    EPD_Update_Display(target_buffer);
}

/**
 * Display Temperature and RGB sequentially
 */
void EPD_Display_Sequence(float temp, uint8_t r, uint8_t g, uint8_t b)
{
    unsigned char target_buffer[15];
    memset(target_buffer, 0x00, 15);
    
    // Convert temperature to digits (for D4, D5, D6)
    int temp_int = (int)(temp >= 0 ? temp * 10.0f + 0.5f : temp * 10.0f - 0.5f);
    if (temp_int < 0) temp_int = -temp_int;
    uint8_t temp_tens = (temp_int / 100) % 10;
    uint8_t temp_ones = (temp_int / 10) % 10;
    uint8_t temp_tenths = temp_int % 10;

    //Display temperature on bottom row - use correct buffer indices
    if(temp_tens > 0) {
        target_buffer[5] = get_digit_pattern(temp_tens)[1];
        target_buffer[6] = get_digit_pattern(temp_tens)[2];
    }
    target_buffer[7] = get_digit_pattern(temp_ones)[1];
    target_buffer[8] = get_digit_pattern(temp_ones)[2];
    target_buffer[9] = get_digit_pattern(temp_tenths)[1];
    target_buffer[10] = get_digit_pattern(temp_tenths)[2];
    
    EPD_Update_Display(target_buffer);
    //memcpy(display_buffer, DSPNUM_1in9_on, 15);
    //EPD_Update_Display(display_buffer);
    //DEV_Delay_ms(2000);  // Increased from 1000ms to 2000ms for better stability with Zigbee active
    //memcpy(display_buffer, DSPNUM_1in9_off, 15);
    //EPD_Update_Display(display_buffer);
}

/**
 * Display startup sequence (000 000 to 999 999)
 */
void EPD_Display_Startup_Sequence(void)
{
    EPD_1in9_init();
    
    unsigned char target_buffer[15];
    // EPD_1in9_lut_DU_WB(); // This LUT setting is handled by EPD_Update_Display

    for (uint8_t i = 0; i <= 2; i++) {
        memset(target_buffer, 0, 15);
        const unsigned char* pat = get_digit_pattern(i);
        
        // Top row: D2, D3, D4
        target_buffer[1] = pat[1];
        target_buffer[2] = pat[2];
        target_buffer[3] = pat[3];
        target_buffer[4] = pat[4];
        target_buffer[5] = pat[5];
        target_buffer[6] = pat[6];
        
        // Bottom row: D5, D6, D7
        target_buffer[7] = pat[7];
        target_buffer[8] = pat[8];
        target_buffer[9] = pat[9];
        target_buffer[10] = pat[10];
        target_buffer[11] = pat[11];
        target_buffer[12] = pat[12];
        
        EPD_Update_Display(target_buffer);
        DEV_Delay_ms(250); // Short delay to let user see the numbers changing
    }

    // Druhá testovací sekvence: 1, 2, 3 nahoře a 4, 5, 6 dole
    memset(target_buffer, 0, 15);
    
    // Horní řádek (pozice D2, D3, D4)
    target_buffer[1] = get_digit_pattern(1)[1];
    target_buffer[2] = get_digit_pattern(1)[2];
    target_buffer[3] = get_digit_pattern(2)[1];
    target_buffer[4] = get_digit_pattern(2)[2];
    target_buffer[5] = get_digit_pattern(3)[1];
    target_buffer[6] = get_digit_pattern(3)[2];
    
    // Spodní řádek (pozice D5, D6, D7)
    target_buffer[7] = get_digit_pattern(4)[1];
    target_buffer[8] = get_digit_pattern(4)[2];
    target_buffer[9] = get_digit_pattern(5)[1];
    target_buffer[10] = get_digit_pattern(5)[2];
    target_buffer[11] = get_digit_pattern(6)[1];
    target_buffer[12] = get_digit_pattern(6)[2];
    
    // Vykreslení a čekání 1 sekund
    EPD_Update_Display(target_buffer);
    DEV_Delay_ms(1000); 
    // Leave display clear at the end
    memcpy(display_buffer, DSPNUM_1in9_off, 15);
    EPD_Update_Display(display_buffer);

    //EPD_1in9_sleep();
}
