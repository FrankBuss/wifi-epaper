#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_task_wdt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "epd.h"

#define FRAME_SIZE (400 * 300 / 8)



// pin numbers

#define SCL_PIN GPIO_NUM_4
#define SDA_PIN GPIO_NUM_16
#define CS_PIN GPIO_NUM_17
#define DC_PIN GPIO_NUM_5
#define RESET_PIN GPIO_NUM_18
#define BUSY_PIN GPIO_NUM_19

// for testing
#define SOFTWARE_SPI


void epdInitHardware(void)
{
	gpio_set_direction(SCL_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(SDA_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(CS_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(DC_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(RESET_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(BUSY_PIN, GPIO_MODE_INPUT);
#ifdef SOFTWARE_SPI
//	platform_gpio_mode(MOSI_PIN, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);
#else
	SET_PERI_REG_MASK(SPI_USER(1), SPI_CS_SETUP|SPI_CS_HOLD|SPI_RD_BYTE_ORDER|SPI_WR_BYTE_ORDER);

	// CPOL = 0
	CLEAR_PERI_REG_MASK(SPI_PIN(1), SPI_IDLE_EDGE);

	// CPHA = 0
	CLEAR_PERI_REG_MASK(SPI_USER(1), SPI_CK_OUT_EDGE);
	CLEAR_PERI_REG_MASK(SPI_USER(1), SPI_FLASH_MODE|SPI_USR_MISO|SPI_USR_ADDR|SPI_USR_COMMAND|SPI_USR_DUMMY);

	// clear Dual or Quad lines transmission mode
	CLEAR_PERI_REG_MASK(SPI_CTRL(1), SPI_QIO_MODE|SPI_DIO_MODE|SPI_DOUT_MODE|SPI_QOUT_MODE);

	// 10 MHz SPI clock
	spi_set_clkdiv(1, 8);

	// configure pins for SPI mode, except CS, because this needs to be controlled by software
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2);

	// TODO: better use the standard spi_master_init function and then platform_spi_select(1, PLATFORM_SPI_SELECT_OFF), when implemented

#endif
}

void epdWriteSclPin(int state)
{
	gpio_set_level(SCL_PIN, state);
}

void epdWriteSdaPin(int state)
{
	gpio_set_level(SDA_PIN, state);
}

void epdWriteCsPin(int state)
{
	gpio_set_level(CS_PIN, state);
}

void epdWriteDcPin(int state)
{
	gpio_set_level(DC_PIN, state);
}

void epdWriteResetPin(int state)
{
	gpio_set_level(RESET_PIN, state);
}

int epdReadBusyPin()
{
	return gpio_get_level(BUSY_PIN);
}

int16_t epdGetTemperature(void)
{
	return 20;  // °C
}

void epdDelayUs(uint32_t us)
{
	ets_delay_us(us);
	esp_task_wdt_feed();
}

void epdDelayMs(uint32_t ms)
{
	while (ms) {
		epdDelayUs(1000);
		ms--;
	}
	esp_task_wdt_feed();

}


#if 0
static uint8_t spiSendReceive(uint8_t c)
{
#ifdef SOFTWARE_SPI
	epdDelayUs(10);
	uint8_t result = 0;
	for (int j = 0; j < 8; j++) {
		if (c & 128) {
			epdWriteMosiPin(1);
		} else {
			epdWriteMosiPin(0);
		}
		c <<= 1;
		epdDelayUs(10);
		epdWriteSpiClockPin(1);
		result <<= 1;
		if (readMisoPin()) {
			result |= 1;
		}
		epdDelayUs(10);
		epdWriteSpiClockPin(0);
	}
	epdDelayUs(10);
	return result;
#else
	return platform_spi_send_recv(1, 8, c);
#endif
}

void epdSpiWrite(uint8_t* data, uint16_t len)
{
	for (int i = 0; i < len; i++) {
		spiSendReceive(data[i]);
	}
}
#endif




void softwareSpi( uint8_t data )
{
	for ( int i = 0; i < 8; i++ ) {
		if ((( data >> (7 - i) ) & 0x01 ) == 1 ) epdWriteSdaPin(1);
		else epdWriteSdaPin(0);
		epdWriteSclPin(1);
		epdWriteSclPin(0);
	}
}

void sendIndexData( uint8_t index, const uint8_t *data, uint32_t len )
{
	epdWriteDcPin(0);
	epdWriteCsPin(0);
	softwareSpi( index );
	epdWriteCsPin(1);
	epdWriteDcPin(1);
	epdWriteCsPin(0);
	for ( int i = 0; i < len; i++ ) softwareSpi( data[ i ] );
	epdWriteCsPin(1);
}

void sendBlankBlack()
{
	epdWriteDcPin(0);
	epdWriteCsPin(0);
	softwareSpi( 0x10 );
	epdWriteCsPin(1);
	epdWriteDcPin(1);
	epdWriteCsPin(0);
	for ( int i = 0; i < FRAME_SIZE; i++ ) softwareSpi( 0x00 );
	epdWriteCsPin(1);
}

void sendBlankRed()
{
	epdWriteDcPin(0);
	epdWriteCsPin(0);
	softwareSpi( 0x13 );
	epdWriteCsPin(1);
	epdWriteDcPin(1);
	epdWriteCsPin(0);
	for ( int i = 0; i < FRAME_SIZE; i++ ) softwareSpi( 0x00 );
	epdWriteCsPin(1);
}


void epdShowImage(const uint8_t* newImage)              //setup function runs once on startup
{
	epdDelayMs( 5 );                         //Delay 5ms
	epdWriteResetPin(1);
	epdDelayMs( 5 );                         //Delay 2ms
	epdWriteResetPin(0);
	epdDelayMs( 10 );
	epdWriteResetPin(1);
	epdDelayMs( 5 );
	epdWriteCsPin(1);
	uint8_t data1[] = { 0x0E };
	sendIndexData( 0x00, data1, 1 );    //Panel Settings
	uint8_t data2[] = { 0x17, 0x17, 0x27 };
	sendIndexData( 0x06, data2, 3 );    //Booster Soft Start Settings
	uint8_t data3[] = { 0x01, 0x90, 0x01, 0x2c };
	sendIndexData( 0x61, data3, 4 );    //Resolution Settings
	uint8_t data4[] = { 0x87 };
	sendIndexData( 0x50, data4, 1 );    //Vcom and data interval setting
	uint8_t data5[] = { 0x88 };
	sendIndexData( 0xe3, data5, 1 );    //Power Saving
	uint8_t data6[] = { 0x02 };
	sendIndexData( 0xE0, data6, 1 );    //Active Temperature
	uint8_t data7[] = { 0x1F };
	sendIndexData( 0xE5, data7, 1 );    //Input Temperature: 25C
	sendIndexData( 0x10, newImage, FRAME_SIZE); //First frame
	sendIndexData( 0x13, newImage + FRAME_SIZE, FRAME_SIZE);   //Second frame
	//sendBlankBlack();
	//sendBlankRed();
	epdDelayMs( 50 );
	uint8_t data8[] = { 0x00 };
	sendIndexData( 0x04, data8, 1 );    //Power on
	epdDelayMs( 5 );
	while( epdReadBusyPin() != 1);
	sendIndexData( 0x12, data8, 1 );    //Display Refresh
	epdDelayMs( 5 );
	while( epdReadBusyPin() != 1);
	sendIndexData( 0x02, data8, 1 );    //Turn off DC/DC
	while( epdReadBusyPin() != 1);
	epdWriteDcPin(0);
	epdWriteCsPin(0);
	epdWriteSdaPin(0);
	epdWriteSclPin(0);
	epdWriteResetPin(0);
}


