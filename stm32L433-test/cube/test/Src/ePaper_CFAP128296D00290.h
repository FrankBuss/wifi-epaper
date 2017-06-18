#ifndef __ePaper_CFAP128296D00290_H__
#define __ePaper_CFAP128296D00290_H__

#include <stdint.h>

#include "stm32l4xx_hal.h"

#define delay HAL_Delay
extern void digitalWrite(int pin, int value);
#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET


//=============================================================================
// "Arduino" example program for Crystalfontz ePaper. 
//
// This project is for the CFAP128296D0-0290 :
//
//   https://www.crystalfontz.com/product/cfap128296d00290
//=============================================================================
// Arduino digital pins used for the e-paper display 
#define EPD_BUSSEL  GPIO_PIN_1
#define EPD_READY   GPIO_PIN_0
#define EPD_RESET   GPIO_PIN_2
#define EPD_DC      GPIO_PIN_3
#define EPD_CS      GPIO_PIN_4
//-----------------------------------------------------------------------------
void ePaper_Init(void);
void Load_Flash_Image_To_Display_RAM(uint16_t width_pixels,
                                     uint16_t height_pixels,
                                     const uint8_t *BW_image,
                                     const uint8_t *red_image);
void ePaper_PowerOff(void);                                     
//=============================================================================
																		 

#endif //  __ePaper_CFAP128296D00290_H__
