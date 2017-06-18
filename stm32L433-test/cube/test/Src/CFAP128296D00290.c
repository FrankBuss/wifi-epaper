//=============================================================================
// "Arduino" example program for Crystalfontz ePaper. 
//
// This project is for the CFAP128296D0-0290 :
//
//   https://www.crystalfontz.com/product/cfap128296d00290
//
// It was written against a Seeduino v4.2 @3.3v. An Arduino UNO modified to
// operate at 3.3v should also work.
//-----------------------------------------------------------------------------
// This is free and unencumbered software released into the public domain.
// 
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
// 
// In jurisdictions that recognize copyright laws, the author or authors
// of this software dedicate any and all copyright interest in the
// software to the public domain. We make this dedication for the benefit
// of the public at large and to the detriment of our heirs and
// successors. We intend this dedication to be an overt act of
// relinquishment in perpetuity of all present and future rights to this
// software under copyright law.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
// OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
// 
// For more information, please refer to <http://unlicense.org/>
//=============================================================================
// Include the ePaper module header.
#include "ePaper_CFAP128296D00290.h"

// Include the images. These images were prepared with "bmp_to_epaper" which
// is available on the Crystalfontz site.
#include "Images_for_CFAP128296D00290.h"
// This module is 128x296 pixels. Make sure that the image files make sense.
#if ((WIDTH_PIXELS != (128)) || (HEIGHT_PIXELS  != (296)))
#error "IMAGE INCLUDE FILE IS WRONG SIZE"
#endif
//===========================================================================
void setup(void)
  {
delay(500);
  //Debug port / Arduino Serial Monitor (optional)
//  Serial.begin(115200);
 
  // Configure the pin directions
/*  pinMode(EPD_READY,  INPUT);
  pinMode(EPD_RESET,  OUTPUT);   
  pinMode(EPD_CS,     OUTPUT);     
  pinMode(EPD_DC,     OUTPUT);     
  pinMode(EPD_BUSSEL, OUTPUT);*/

  // Set output pins' default state
  digitalWrite(EPD_RESET,  HIGH); //active low
  digitalWrite(EPD_CS,     HIGH); //active low
  digitalWrite(EPD_DC,     HIGH); //active low 
  digitalWrite(EPD_BUSSEL, LOW);  // 4 wire mode
  
  //Set up SPI interface
  /*SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  SPI.begin();*/

  //Initialize the display controller hardware 
  ePaper_Init();
  }
//=============================================================================
#define SHUTDOWN_BETWEEN_UPDATES (0)
void loop()
  {
  Load_Flash_Image_To_Display_RAM(128,
                                  296,
                                  CFAP128296D0_0290_Splash_Black_Layer_1BPP,
                                  CFAP128296D0_0290_Splash_Red_Layer_1BPP);
  
  //Wait for the refresh to complete (~11sec).
  while(0 == HAL_GPIO_ReadPin(GPIOB, EPD_READY));

  #if(SHUTDOWN_BETWEEN_UPDATES)
    //Enter off / low power state
    ePaper_PowerOff();
  #endif
  
  //Pause to view
  delay(5000);

  #if(SHUTDOWN_BETWEEN_UPDATES)
    //Wake it up
    ePaper_Init();
  #endif

  Load_Flash_Image_To_Display_RAM(128,
                                  296,
                                  Text_Demo_Black_Layer_1BPP,
                                  Text_Demo_Red_Layer_1BPP);

  //Wait for the refresh to complete (~11sec).
  while(0 == HAL_GPIO_ReadPin(GPIOB, EPD_READY));

  #if(SHUTDOWN_BETWEEN_UPDATES)
    //Enter off / low power state
    ePaper_PowerOff();
  #endif

  //Pause to view
  delay(5000);

  #if(SHUTDOWN_BETWEEN_UPDATES)
    //Wake it up
    ePaper_Init();
  #endif
  }
//=============================================================================
