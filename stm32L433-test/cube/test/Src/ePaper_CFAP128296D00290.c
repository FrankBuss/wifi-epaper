//=============================================================================
// "Arduino" example program for Crystalfontz ePaper. 
//
// This project is for the CFAP128296C0-0290 :
//
//   https://www.crystalfontz.com/product/cfap128296d00290
//=============================================================================
//#include <SPI.h>

#include <stdint.h>

#include "ePaper_CFAP128296D00290.h"
#include "LUTs_for_CFAP128296D00290.h"
#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi2;


//SPI access macros
#define ePaper_RST_0  (digitalWrite(EPD_RESET, LOW))
#define ePaper_RST_1  (digitalWrite(EPD_RESET, HIGH))
#define ePaper_CS_0   (digitalWrite(EPD_CS, LOW))
#define ePaper_CS_1   (digitalWrite(EPD_CS, HIGH))
#define ePaper_DC_0   (digitalWrite(EPD_DC, LOW))
#define ePaper_DC_1   (digitalWrite(EPD_DC, HIGH))
#define ePaper_BS_0   (digitalWrite(EPD_BUSSEL, LOW))
#define ePaper_BS_1   (digitalWrite(EPD_BUSSEL, HIGH))

void SPI_transfer(uint8_t byte)
{
	HAL_SPI_Transmit(&hspi2, &byte, 1, 10000);
}


//=============================================================================
void ePaper_WriteCMD(uint8_t command)
  {
  //Make sure the dsplay is not busy before starting a new command.
  while(0 == HAL_GPIO_ReadPin(GPIOB, EPD_READY));
  //Select the controller   
  ePaper_CS_0;
  //Aim at the command register
  ePaper_DC_0;
  SPI_transfer(command);
  //Deselect the controller
  ePaper_CS_1;
  }
//=============================================================================
// command in data[0]
// parameters in data[1]. . .data[datalen-1]
#define PTR_AND_SIZ(x) x,sizeof(x)
void WriteCMD_StringFlash(const uint8_t *data, uint8_t datalen)
  {
  //Index into *image, that works with pgm_read_byte()
  uint8_t
    index;
  index=0;

  //Make sure the dsplay is not busy before starting a new command.
  while(0 == HAL_GPIO_ReadPin(GPIOB, EPD_READY));
  //Select the controller   
  ePaper_CS_0;
  //The first byte in the string is the command, aim at the command register
  ePaper_DC_0;
  SPI_transfer(data[index++]);
  //Remember that we wrote the command that was in data[0]
  datalen--;

  //Remaining bytes are data, aim at the data register
  ePaper_DC_1;
  for(uint8_t i= 0;i<datalen;i++)
    {
    SPI_transfer(data[index++]);
    }
  //Deslect the controller   
  ePaper_CS_1;
  }
//=============================================================================	
void Load_Flash_Image_To_Display_RAM(uint16_t width_pixels,
                                     uint16_t height_pixels,
                                     const uint8_t *BW_image,
                                     const uint8_t *red_image)
  {
  //Index into *image, that works with pgm_read_byte()
  uint16_t
    index;
  index=0;
  
  //Get width_bytes from width_pixel, rounding up
  uint8_t
    width_bytes;
  width_bytes=(width_pixels+7)>>3;  
  
  //Make sure the display is not busy before starting a new command.
  while(0 == HAL_GPIO_ReadPin(GPIOB, EPD_READY));
  //Select the controller   
  ePaper_CS_0;

  //Aim at the command register
  ePaper_DC_0;
  //Write the command: DATA START TRANSMISSION 1 (DTM1) (R10H)
  //  Display Start Transmission 1
  //  (DTM1, White/Black Data)
  //
  // This command starts transmitting data and write them into SRAM. To complete
  // data transmission, command DSP (Data transmission Stop) must be issued. Then
  // the chip will start to send data/VCOM for panel.
  //  * In B/W mode, this command writes “OLD” data to SRAM.
  //  * In B/W/Red mode, this command writes “B/W” data to SRAM.
  SPI_transfer(0x10);
  //Pump out the BW data.
  ePaper_DC_1;
  index=0;
  for(uint16_t y=0;y<height_pixels;y++)
    {
    for(uint8_t x=0;x<width_bytes;x++)
      {
      SPI_transfer(BW_image[index]);
      index++;
      }
    }
 
  //Aim back to the command register
  ePaper_DC_0;
  //Write the command: DATA START TRANSMISSION 2 (DTM2) (R13H)
  //  Display Start transmission 2
  //  (DTM2, Red Data)
  //
  // This command starts transmitting data and write them into SRAM. To complete
  // data transmission, command DSP (Data transmission Stop) must be issued. Then
  // the chip will start to send data/VCOM for panel.
  //  * In B/W mode, this command writes “NEW” data to SRAM.
  //  * In B/W/Red mode, this command writes “RED” data to SRAM.
  SPI_transfer(0x13);
  //Pump out the RED data.
  ePaper_DC_1;
  index=0;
  for(uint16_t y=0;y<height_pixels;y++)
    {
    for(uint8_t x=0;x<width_bytes;x++)
      {
      SPI_transfer(red_image[index]);
      index++;
      }
    }
  
  //Aim back at the command register
  ePaper_DC_0;
  //Write the command: DATA STOP (DSP) (R11H)
  SPI_transfer(0x11);
  //Write the command: Display Refresh (DRF)   
  SPI_transfer(0x12);
  //Deslect the controller   
  ePaper_CS_1;
  }
//=============================================================================
const uint8_t Power_Setting_PWR_On[]  =
   {0x01,0x03,0x00,0x0A,0x00,0x03};
const uint8_t Booster_Soft_Start_BTST[]  =
   {0x06,0x17,0x17,0x17};
const uint8_t Power_ON_PON[]  =
   {0x04};
const uint8_t Panel_Setting_PSR[]  =
   {0x00,0x83};
  // 1000 0011 = Crystalfontz
  // RREB ULDR
  // |||| ||||-- RST_N *1 = run, 0 for software reset
  // |||| |||--- SHD_N *1 = Booster on, 0 = Booster off
  // |||| ||---- SHL   *1 = scan right, 0 = scan left
  // |||| |----- UD     1 = scan up, *0 = scan down
  // ||||------- BWR    0 = Black/White/Red (Runs LU1 & LU2)
  // |||               *1 = Black/White (Runs LU1 only)
  // |||-------- REG_EN 1 = LUT from register, *0 = LUT from OTP
  // ||--------- RES1:RES0 (over-ridden by 0x61)
  //                 00 = 96x230
  //                 01 = 96x252
  //                *10 = 128x296
  //                 11 = 160x296
const uint8_t Vcom_and_data_interval_setting_CDI[]  =
  {0x50,0x87};
  // 1000 0111 = Crystalfontz
  // BBRB CCCC
  // |||| ||||-- CDI: 0011 = default
  // ||||------- DDX[0]: Black Data Polarity 1=ink, 0=white
  // |||-------- DDX[1]: Red Data Polarity   1=ink, 0=white
  // ||--------- VBD: Border Data Selection
  //             (by experiment for DDX = 00)
  //               00=really light grey
  //               01=muddy red
  //               10=white <<<<
  //               11=black

const uint8_t PLL_control_PLL[]  =
  {0x30,0x29};
const uint8_t Resolution_setting_TRES[]  =
  {0x61,128,296>>8,296&0xFF};
const uint8_t VCM_DC_Setting_VDCS_On[]  =
  {0x82,0x0A};
//-----------------------------------------------------------------------------
void ePaper_Init(void)
  {
  //Select 4-wire SPI mode.
  ePaper_BS_0;

  //Give the controller a hardware reset
  ePaper_RST_0;
  delay(1);
  ePaper_RST_1;
  delay(1);
  
  WriteCMD_StringFlash(PTR_AND_SIZ(Power_Setting_PWR_On));
  WriteCMD_StringFlash(PTR_AND_SIZ(Booster_Soft_Start_BTST));
  WriteCMD_StringFlash(PTR_AND_SIZ(Power_ON_PON));
  WriteCMD_StringFlash(PTR_AND_SIZ(Panel_Setting_PSR));
  WriteCMD_StringFlash(PTR_AND_SIZ(Vcom_and_data_interval_setting_CDI));
  WriteCMD_StringFlash(PTR_AND_SIZ(PLL_control_PLL));
  WriteCMD_StringFlash(PTR_AND_SIZ(Resolution_setting_TRES));
  WriteCMD_StringFlash(PTR_AND_SIZ(VCM_DC_Setting_VDCS_On));

  //Load the look-up-tables (LUTs), which contain the waveforms
  //that instruct the controller how to jiggle the ink particals
  //into the correct places to produce the image.
  WriteCMD_StringFlash(PTR_AND_SIZ(VCOM_LUT_LUTC));
  WriteCMD_StringFlash(PTR_AND_SIZ(W2W_LUT_LUTWW));
  WriteCMD_StringFlash(PTR_AND_SIZ(B2W_LUT_LUTBW_LUTR));
  WriteCMD_StringFlash(PTR_AND_SIZ(W2B_LUT_LUTWB_LUTW));
  WriteCMD_StringFlash(PTR_AND_SIZ(B2B_LUT_LUTBB_LUTB));
  }
//=============================================================================
const uint8_t Vcom_and_data_interval_setting_CDI_Off[]  =
   {0x50,0x17};
const uint8_t VCM_DC_Setting_VDCS_Off[]  =
   {0x82,0x00};
const uint8_t Power_Setting_PWR_Off[]  =
   {0x01,0x00,0x00,0x00,0x00,0x00};
const uint8_t Power_OFF_POF[]  =
   {0x02};
const uint8_t Deep_sleep_DSLP[]  =
   {0x07,0xA5};
//-----------------------------------------------------------------------------
void ePaper_PowerOff(void)
  {
  WriteCMD_StringFlash(PTR_AND_SIZ(Vcom_and_data_interval_setting_CDI_Off));
  WriteCMD_StringFlash(PTR_AND_SIZ(VCM_DC_Setting_VDCS_Off));
  WriteCMD_StringFlash(PTR_AND_SIZ(Power_Setting_PWR_Off));
  WriteCMD_StringFlash(PTR_AND_SIZ(Power_OFF_POF));
  WriteCMD_StringFlash(PTR_AND_SIZ(Deep_sleep_DSLP));
  }
//=============================================================================   
