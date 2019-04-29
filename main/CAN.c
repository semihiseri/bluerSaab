/*
 ----------------------------------------------------------------------------------
 CAN.cpp
 CONTROLLER AREA NETWORK (CAN 2.0A STANDARD ID)
 CAN BUS library for Wiring/Arduino - Version 1.1
 ADAPTED FROM http://www.kreatives-chaos.com
 By IGOR REAL (16 - 05 - 2011)
 ----------------------------------------------------------------------------------
 */
/*
 Name:
 Parameters(type):
 Description:
 Example:
 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"

#include "sdkconfig.h" // why is this necessary?




#include "CAN.h"

#define DEBUGMODE	0


/******************************************************************************
 * Variables
 ******************************************************************************/
//CANClass CAN;

msgCAN CAN_TxMsg;
msgCAN CAN_RxMsg;


/******************************************************************************
 * Constructors
 ******************************************************************************/

#define MCP2515_CS	14
#define MCP2515_SCK	33
#define MCP2515_MOSI	32

#define MCP2515_MISO	35
#define MCP2515_INT	13

spi_device_handle_t spi;

/******************************************************************************
 * PUBLIC METHODS
 ******************************************************************************/
void CAN_begin(uint16_t speed)
{
    
#if (DEBUGMODE==1)
    printf("-- Constructor Can(uint16_t speed) --\n");
#endif


	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = 	(((uint64_t) 1)<<MCP2515_CS);
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

	esp_err_t ret;
	spi_bus_config_t buscfg={
		.miso_io_num=MCP2515_MISO,
		.mosi_io_num=MCP2515_MOSI,
		.sclk_io_num=MCP2515_SCK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=16*320*2+8 // This can be adjusted, I guess. But not necessary.
		};

	spi_device_interface_config_t devcfg={
		.clock_speed_hz=8*1000*1000,           //Clock out at 8 MHz - maybe slower?
		.mode=0,                                //SPI mode 0
		.spics_io_num=-1,//MCP2515_CS,               //CS pin - we want total control, hence -1.
		.queue_size=7,                          //We want to be able to queue 7 transactions at a time
		};

	//Initialize the SPI bus
	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	ESP_ERROR_CHECK(ret);

	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);


	

    
    // We activate the SPI Arduino as Master and Fosc/2=8 MHz
//    SPCR = (1<<SPE)|(1<<MSTR) | (0<<SPR1)|(0<<SPR0); // <-------------- ARDUINO Specific register!! Must kill!!
//    SPSR = (1<<SPI2X); // <-------------------------------------------- ARDUINO Specific register!! Must kill!!
#if (DEBUGMODE==1)
    printf("SPI=8 Mhz\n");
#endif
    
    
    
    // reset MCP2515 by software reset.
    // After this he is in configuration mode.
    RESET(MCP2515_CS);
    spi_putc(SPI_RESET);
    SET(MCP2515_CS);
    
    // wait a little bit until the MCP2515 has restarted
//    _delay_us(10);
	vTaskDelay(1 / portTICK_RATE_MS);
    
    
    
    switch(speed)
    {
        case 47:
            /*
             Supposed to use...?
             SJW = 1
             SP% ~= 75
            */
            //Original CNF values
             mcp2515_write_register(CNF1,0xC7);
             mcp2515_write_register(CNF2,0xBE);
             mcp2515_write_register(CNF3,0x04);
            /*
             T1   = 16
             T2   = 5
             BTQ  = 21
             SP%  = 76.19
             SJW  = 4
             Err% = 0
             */
            
            /*
             // Version 1
             mcp2515_write_register(CNF1,0x4B);
             mcp2515_write_register(CNF2,0xF1);
             mcp2515_write_register(CNF3,0x03);
             
             T1   = 14
             T2   = 7
             BTQ  = 21
             SP%  = 66.67
             SJW  = 4
             Err% = 0
             
             
             // Version 2
             mcp2515_write_register(CNF1,0xC7);
             mcp2515_write_register(CNF2,0xFE);
             mcp2515_write_register(CNF3,0x44);
             
             T1   = 16
             T2   = 8
             BTQ  = 24
             SP%  = 66.67
             SJW  = 1
             Err% = 0
             */
            /*
            // Version 3
            mcp2515_write_register(CNF1,0xC7);
            mcp2515_write_register(CNF2,0xBE);
            mcp2515_write_register(CNF3,0x44);
             T1   = 10
             T2   = 4
             BTQ  = 14
             SP%  = 71.4
             SJW  = 2
             Err% = 0
             */
            
#if (DEBUGMODE==1)
            printf("Speed = 47.619Kbps\n");
#endif
            break;
            
        case 1:
            mcp2515_write_register(CNF1,0x00);
            mcp2515_write_register(CNF2,0x90);
            mcp2515_write_register(CNF3,0x02);
#if (DEBUGMODE==1)
            printf("Speed = 1Mbps\n");
#endif
            break;
            
        case 500:
            mcp2515_write_register(CNF1,0x01);
            mcp2515_write_register(CNF2,0x90);
            mcp2515_write_register(CNF3,0x02);
#if (DEBUGMODE==1)
            printf("Speed = 500Kbps\n");
#endif
            break;
            
        case 250:
            mcp2515_write_register(CNF1,0x01);
            mcp2515_write_register(CNF2,0xB8);
            mcp2515_write_register(CNF3,0x05);
#if (DEBUGMODE==1)
            printf("Speed = 250Kbps\n");
#endif
            break;
            
        case 125:
            mcp2515_write_register(CNF1,0x07);
            mcp2515_write_register(CNF2,0x90);
            mcp2515_write_register(CNF3,0x02);
#if (DEBUGMODE==1)
            printf("Speed = 125Kbps\n");
#endif
            break;
            
        case 100:
            mcp2515_write_register(CNF1,0x03);
            mcp2515_write_register(CNF2,0xBA);
            mcp2515_write_register(CNF3,0x07);
#if (DEBUGMODE==1)
            printf("Speed = 100Kbps\n");
#endif
            break;
            
        default:
            mcp2515_write_register(CNF1,0x00);
            mcp2515_write_register(CNF2,0x90);
            mcp2515_write_register(CNF3,0x02);
#if (DEBUGMODE==1)
            printf("Speed = Default\n");
#endif
            break;
            
    }
    
    
    
    
    //Activate RX Interruption
    mcp2515_write_register(CANINTE,(1<<RX1IE)|(1<<RX0IE)); //The two buffers activated interrupt pin
    
    //Filters
    //Buffer 0: All Messages and Rollover=>If buffer 0 full, send to buffer 1
    mcp2515_write_register(RXB0CTRL,(1<<RXM1)|(1<<RXM0)|(1<<BUKT)); //RXM1 & RXM0 the filter/mask off+Rollover
    //Buffer 1: All Messages
    mcp2515_write_register(RXB1CTRL,(1<<RXM1)|(1<<RXM0)); //RXM1 & RXM0 the filter/mask off
    
    //Clear reception mask bits
    mcp2515_write_register( RXM0SIDH, 0 );
    mcp2515_write_register( RXM0SIDL, 0 );
    mcp2515_write_register( RXM0EID8, 0 );
    mcp2515_write_register( RXM0EID0, 0 );
    mcp2515_write_register( RXM1SIDH, 0 );
    mcp2515_write_register( RXM1SIDL, 0 );
    mcp2515_write_register( RXM1EID8, 0 );
    mcp2515_write_register( RXM1EID0, 0 );
    
    //Turn the LED on the board connected to RX0BF / RX1BF when a msg in the buffer
    mcp2515_write_register( BFPCTRL, 0b00001111 );
    
    //Pass the MCP2515 to normal mode and One Shot Mode 0b00001000
    // OSM switched off
    
    mcp2515_write_register(CANCTRL, 0);
    
    //Initialize buffer
    _CAN_RX_BUFFER.head=0;
    _CAN_RX_BUFFER.tail=0;
    
#if (DEBUGMODE==1)
    printf("-- End Constructor Can(uint16_t speed) --\n");
#endif
    
    
}
// ----------------------------------------------------------------------------
/*
 Name:send(message)
 Parameters(type):
	message(*mesCAN):message to be sent
 Description:
	It sends through the bus the message passed by reference
 Returns:
	0xFF: if 2515's tx-buffer is full
	0x01: if when 2515's TxBuffer[0] used
	0x02: if when 2515's TxBuffer[1] used
	0x04: if when 2515's TxBuffer[2] used
 Example:
	uint8_t count = 0;
	while(CAN.send(!CAN_TxMsg)==0xFF && count < 15);
 
 */
uint8_t CAN_send(msgCAN *message)
{
    
    
#if (DEBUGMODE==1)
    printf("-- uint8_t CANClass::send(msgCAN *message) --\n");
#endif
    
    uint8_t status = mcp2515_read_status(SPI_READ_STATUS);
    
    /* Statusbyte:
     *
     * Bit	Function
     *  2	TXB0CNTRL.TXREQ
     *  4	TXB1CNTRL.TXREQ
     *  6	TXB2CNTRL.TXREQ
     */
    uint8_t address;
    uint8_t t;
    
#if (DEBUGMODE==1)
    printf("MCP2515 Status=");
    printf("%x\n",status);
    printf("Mask to check Buffer=");
    printf("%x\n", ((status & 0b11000000)>>6)&0b00000011);
#endif

    if (bit_is_clear(status, 2)) {
        address = 0x00;
    }
    else if (bit_is_clear(status, 4)) {
        address = 0x02;
    }
    else if (bit_is_clear(status, 6)) {
        address = 0x04;
    }
    else {
        // all buffer used => could not send message
        return 0xFF;
    }
    
    RESET(MCP2515_CS);
    spi_putc(SPI_WRITE_TX | address);
    
    spi_putc(message->id >> 3);
    spi_putc(message->id << 5);
    
    spi_putc(0);
    spi_putc(0);
    
    uint8_t length = message->header.length & 0x0f;
    
    if (message->header.rtr) {
        // a rtr-frame has a length, but contains no data
        spi_putc((1<<RTR) | length);
    }
    else {
        // set message length
        spi_putc(length);
        
        // data
        for (t=0;t<length;t++) {
            spi_putc(message->data[t]);
        }
    }
    SET(MCP2515_CS);
    
//    _delay_us(1);
	vTaskDelay(1 / portTICK_RATE_MS);
    
    // send message
    RESET(MCP2515_CS);
    address = (address == 0) ? 1 : address;
    spi_putc(SPI_RTS | address);
    SET(MCP2515_CS);
    
    
#if (DEBUGMODE==1)
    printf("-- END uint8_t CANClass::send(msgCAN *message) --\n");
#endif
    
    
    
    return address;
}
// ----------------------------------------------------------------------------
/*
 Name:ReadFromDevice(message)
 Parameters(type):
	message(*msgCAN): Menssage passed by reference to be filled with the new
 message coming fron the 2515 drvier
 Description:
	Receives by parameter a struct msgCAN
 Returns(uint8_t):
	last 3 bits of 2515's status. More in the
 Example:
	if(CAN.ReadFromDevice(&message))
	{
	}
 */
uint8_t CAN_ReadFromDevice(msgCAN *message)
{
    
    
#if (DEBUGMODE==1)
    printf("-- START uint8_t ReadFromDevice(msgCAN *message) --\n");
#endif
    
    //	static uint8_t previousBuffer;
    
    // read status
    uint8_t status = mcp2515_read_status(SPI_RX_STATUS);
    uint8_t addr;
    uint8_t t;
    
#if (DEBUGMODE==1)
    printf("MCP2515 Status=");
    printf("%x\n",status);
    printf("Mask to check Buffer=");
    printf("%x\n", ((status & 0b11000000)>>6)&0b00000011);
#endif
    
    /* This piece of code sometimes causes us to read from the wrong buffer
     
     if ( (((status & 0b11000000)>>6)&0b00000011) >2 )
     {
     addr=SPI_READ_RX | (previousBuffer++ & 0x01)<<2;
     
     #if (DEBUGMODE==1)
     Serial.println("Dos buffer con datos");
     Serial.print("addr=");
     Serial.println(addr,HEX);
     Serial.print("previousBuffer=");
     Serial.println(previousBuffer,DEC);
     
     #endif
     }
     else
     */
    if (bit_is_set(status,6))
    {
        // message in buffer 0
        addr = SPI_READ_RX;
        
#if (DEBUGMODE==1)
        printf("Read From Buffer 0\n");
        printf("addr=");
        printf("%x\n", addr);
#endif
    }
    else if (bit_is_set(status,7))
    {
        // message in buffer 1
        addr = SPI_READ_RX | 0x04;
        
#if (DEBUGMODE==1)
        printf("Read From Buffer 1\n");
        printf("addr=");
        printf("%x\n", addr);
#endif
    }
    else {
        // Error: no message available
        return 0;
    }
    
    RESET(MCP2515_CS);
    spi_putc(addr);
    
    // read id
    message->id  = (uint16_t) spi_putc(0xff) << 3;
    message->id |=            spi_putc(0xff) >> 5;
    
    spi_putc(0xff);
    spi_putc(0xff);
    
    // read DLC
    uint8_t length = spi_putc(0xff) & 0x0f;
    
    message->header.length = length;
    message->header.rtr = (bit_is_set(status, 3)) ? 1 : 0;
    
    // read data
    for (t=0;t<length;t++) {
        message->data[t] = spi_putc(0xff);
    }
    SET(MCP2515_CS);
    
    /*
     // clear interrupt flag
     if (bit_is_set(status, 6)) {
     mcp2515_bit_modify(CANINTF, (1<<RX0IF), 0);
     }
     else {
     mcp2515_bit_modify(CANINTF, (1<<RX1IF), 0);
     }
     */
    
    
    
#if (DEBUGMODE==1)
    printf("Return = ");
    printf("%d\n", (status & 0x07) + 1);
    printf("-- END uint8_t Can::ReadFromDevice(msgCAN *message) --\n");
#endif
    
    
    return (status & 0x07) + 1;
    
    
    
}
// ----------------------------------------------------------------------------
/*
 Name: CheckNew()
 Parameters(type):
	None
 Description:
	Polls interrupt bit
 Returns:
	0: if there is not messages waiting in the converter
	1: if there is
 Example:
	if (CAN.ChekNew())
 
 */
uint8_t CAN_CheckNew(void)
{
    return (!IS_SET(MCP2515_INT));
}
// ----------------------------------------------------------------------------
/*
 Name: SetMode(mode)
 
 Parameters(type):
	uint8_t mode
	
 Description:
	The MCP2515 has five modes of operation. This function configure:
 1. Listen-only mode
 2. Loopback mode
 3. Sleep mode
 4. Normal mode
	
 Returns:
	Nothing
	
 Example:
	
 */
void CAN_SetMode(uint8_t mode)
{
    uint8_t reg = 0;
    
    if (mode == LISTEN_ONLY_MODE) {
        reg = (0<<REQOP2)|(1<<REQOP1)|(1<<REQOP0);
    }
    else if (mode == LOOPBACK_MODE) {
        reg = (0<<REQOP2)|(1<<REQOP1)|(0<<REQOP0);
    }
    else if (mode == SLEEP_MODE) {
        reg = (0<<REQOP2)|(0<<REQOP1)|(1<<REQOP0);
    }
    else if (mode == NORMAL_MODE) {
        reg = (0<<REQOP2)|(0<<REQOP1)|(0<<REQOP0);
    }
    
    // Set the new mode
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), reg);
    // Wait until the mode has been changed
    while ((mcp2515_read_register(CANSTAT) & 0xe0) != reg) {
        // wait for the new mode to become active
    }
    
}
// ----------------------------------------------------------------------------
/*
 Name: SetFilter
 
 Parameters(type):
	uint8_t mode
	
 Description:
 
	
 Returns:
	Nothing
	
 Example:
	
 */
void CAN_SetFilters(uint16_t *Filters,uint16_t *Masks)
{
    
#if (DEBUGMODE==1)
    printf("-- void CANClass::SetFilters(uint16_t *Filters) --\n");
#endif
    
    
    //Mask=0xE0=0b11100000  (bits 7-5)
    //Set Config Mode=100 (bits 7-5)
    mcp2515_bit_modify(CANCTRL,0xE0,(1<<REQOP2));
    
    //Wait until changed to Config Mode
    while ((mcp2515_read_register(CANSTAT) & 0xE0) != (1<<REQOP2));
    
    //Buffer RXB0 => Filter 0-1 & Mask 0
    mcp2515_write_register( RXF0SIDH, (uint8_t)(Filters[0]>>3) );
    mcp2515_write_register( RXF0SIDL, (uint8_t)(Filters[0]<<5) );
    
    mcp2515_write_register( RXF1SIDH, (uint8_t)(Filters[1]>>3) );
    mcp2515_write_register( RXF1SIDL, (uint8_t)(Filters[1]<<5) );
    
    //Mask 0
    mcp2515_write_register( RXM0SIDH, (uint8_t)(Masks[0]>>3) );
    mcp2515_write_register( RXM0SIDL, (uint8_t)(Masks[0]<<5) );
    //---------------------------------------------------------------
    
    //Buffer RXB1 => Filter 2-5 & Mask 1
    mcp2515_write_register( RXF2SIDH, (uint8_t)(Filters[2]>>3) );
    mcp2515_write_register( RXF2SIDL, (uint8_t)(Filters[2]<<5) );
    
    mcp2515_write_register( RXF3SIDH, (uint8_t)(Filters[3]>>3) );
    mcp2515_write_register( RXF3SIDL, (uint8_t)(Filters[3]<<5) );
    
    mcp2515_write_register( RXF4SIDH, (uint8_t)(Filters[4]>>3) );
    mcp2515_write_register( RXF4SIDL, (uint8_t)(Filters[4]<<5) );
    
    mcp2515_write_register( RXF5SIDH, (uint8_t)(Filters[5]>>3) );
    mcp2515_write_register( RXF5SIDL, (uint8_t)(Filters[5]<<5) );
    
    // Mask1
    mcp2515_write_register( RXM1SIDH, (uint8_t)(Masks[1]>>3) );
    mcp2515_write_register( RXM1SIDL, (uint8_t)(Masks[1]<<5) );
    
    
    //Buffer configuration
    //Bufer 0
    mcp2515_write_register(RXB0CTRL,(0<<RXM1)|(1<<RXM0)|(1<<BUKT));
    //Bufer 1
    mcp2515_write_register(RXB1CTRL,(0<<RXM1)|(1<<RXM0));
    
    
#if (DEBUGMODE==1)
    printf("Filter 0 = ");
    printf("%x\n", Filters[0]); // Binary options are removed; printf can't format binary
    
    printf("Filter 1 = ");
    printf("%x\n", Filters[1]);
    
    printf("Filter 2 = ");
    printf("%x\n", Filters[2]);
    
    printf("Filter 3 = ");
    printf("%x\n", Filters[3]);
    
    printf("Filter 4 = ");
    printf("%x\n", Filters[4]);
    
    printf("Filter 5 = ");
    printf("%x\n", Filters[5]);
    
    printf("Mask 0 = ");
    printf("%x\n", Masks[0]);
    
    printf("Mask 1 = ");
    printf("%x\n", Masks[1]);
    
    printf("RXB0CTRL = ");
    printf("%x\n", mcp2515_read_register(RXB0CTRL));
    printf("RXB1CTRL = ");
    printf("%x\n", mcp2515_read_register(RXB1CTRL));
    printf("----------------------\n");
    
    printf("RXF0SIDH = ");
    printf("%x\n", mcp2515_read_register(RXF0SIDH));
    printf("RXF0SIDL = ");
    printf("%x\n", mcp2515_read_register(RXF0SIDL));
    printf("RXF1SIDH = ");
    printf("%x\n", mcp2515_read_register(RXF1SIDH));
    printf("RXF1SIDL = ");
    printf("%x\n", mcp2515_read_register(RXF1SIDL));
    printf("RXF2SIDH = ");
    printf("%x\n", mcp2515_read_register(RXF2SIDH));
    printf("RXF2SIDL = ");
    printf("%x\n", mcp2515_read_register(RXF2SIDL));
    printf("RXF3SIDH = ");
    printf("%x\n", mcp2515_read_register(RXF3SIDH));
    printf("RXF3SIDL = ");
    printf("%x\n", mcp2515_read_register(RXF3SIDL));
    printf("RXF4SIDH = ");
    printf("%x\n", mcp2515_read_register(RXF4SIDH));
    printf("RXF4SIDL = ");
    printf("%x\n", mcp2515_read_register(RXF4SIDL));
    printf("RXF5SIDH = ");
    printf("%x\n", mcp2515_read_register(RXF5SIDH));
    printf("RXF5SIDL = ");
    printf("%x\n", mcp2515_read_register(RXF5SIDL));
    printf("----------------------\n");
    
    printf("RXM0SIDH = ");
    printf("%d\n", mcp2515_read_register(RXM0SIDH));
    printf("RXM0SIDL = ");
    printf("%x\n", mcp2515_read_register(RXM0SIDL));
    printf("RXM1SIDH = ");
    printf("%x\n", mcp2515_read_register(RXM1SIDH));
    printf("RXM1SIDL = ");
    printf("%x\n", mcp2515_read_register(RXM1SIDL));
    
    
    
#endif
    
    //Normal Operation Mode
    mcp2515_bit_modify(CANCTRL,0xE0,0);
    
#if (DEBUGMODE==1)
    printf("-- END void CANClass::SetFilters(uint16_t *Filters) --\n");
#endif
    
    
}
// ----------------------------------------------------------------------------






/******************************************************************************
 * PRIVATE METHODS
 ******************************************************************************/

// -------------------------------------------------------------------------
uint8_t spi_putc(uint8_t data)
{
	esp_err_t ret;
	spi_transaction_t t;

	memset(&t, 0, sizeof(t));       //Zero out the transaction

	t.length=8;                 //Len is in bytes, transaction length is in bits.
	t.tx_data[0]=data;               //Data

	t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

	ret = spi_device_transmit(spi, &t);
	assert( ret == ESP_OK );

	//printf("putc %d return: %x\n", ret, *(uint32_t*)t.rx_data);


	return (uint8_t) (*(uint32_t*)t.rx_data)&0xFF; // This will fail, for sure.
}

// -------------------------------------------------------------------------
void mcp2515_write_register( uint8_t adress, uint8_t data )
{
    RESET(MCP2515_CS);
    
    spi_putc(SPI_WRITE);
    spi_putc(adress);
    spi_putc(data);
    
    SET(MCP2515_CS);
}
// ----------------------------------------------------------------------------
uint8_t mcp2515_read_status(uint8_t type)
{
    uint8_t data;
    
    RESET(MCP2515_CS);
    
    spi_putc(type);
    data = spi_putc(0xff);
    
    SET(MCP2515_CS);
    
    return data;
}
// -------------------------------------------------------------------------
void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
    RESET(MCP2515_CS);
    
    spi_putc(SPI_BIT_MODIFY);
    spi_putc(adress);
    spi_putc(mask);
    spi_putc(data);
    
    SET(MCP2515_CS);
}
// ----------------------------------------------------------------------------
uint8_t mcp2515_check_free_buffer(void)
{
    uint8_t status = mcp2515_read_status(SPI_READ_STATUS);
    
    if ((status & 0x54) == 0x54) {
        // all buffers used
        return false;
    }
    
    return true;
}
// ----------------------------------------------------------------------------
uint8_t mcp2515_read_register(uint8_t adress)
{
    uint8_t data;
    
    RESET(MCP2515_CS);
    
    spi_putc(SPI_READ);
    spi_putc(adress);
    
    data = spi_putc(0xff);
    
    SET(MCP2515_CS);
    
    return data;
}
// ----------------------------------------------------------------------------
/*
 Name: store(message)
 Parameters(type):
	message(msgCAN*); Message to sotre in circular buffer
 Description:
	it stores the message given in the circular buffer
 Returns:
	None
 Example:
	CAN.store(&CAN_RxMsg);
 
 */
void CAN_store(msgCAN *message)
{
    
    int i=(_CAN_RX_BUFFER.head+1)%RX_CAN_BUFFER_SIZE;
    
    //Must be stored in the buffer, as long as're from behind the tail
    //We see example if the buffer size is 10:
    //The rest of 1/10=1, meter data CAN_RX_BUFFER.buffer[0]; i=1
    //The rest of 2/10=2, meter data CAN_RX_BUFFER.buffer[1]; i=2
    //...
    //The rest of 10/10=0, BUFFER FULL. Does nothing.
    
    if (i!=_CAN_RX_BUFFER.tail)
    {
        _CAN_RX_BUFFER.buffer[_CAN_RX_BUFFER.head]=*message;
        //increase the current position of the circular buffer
        _CAN_RX_BUFFER.head=i;
        
    }else{
        
        //todo
        
    }
    
    
}
// ----------------------------------------------------------------------------
/*
 Name:available()
 Parameters(type):
	None
 Returns
	0 if there is not available messages in bus
	>0 : How many
 Description:
	Returns how many messages are available in the circular buffer
 Example:
	if(CAN.available() > 0)
	{
 
	}
 
 */
uint8_t CAN_available(void)
{
    return ((RX_CAN_BUFFER_SIZE+_CAN_RX_BUFFER.head-_CAN_RX_BUFFER.tail)%RX_CAN_BUFFER_SIZE);
}
// ----------------------------------------------------------------------------
/*
 Name:read(message)
 Parameters(type):
	message(*msgCAN): Message passed by reference t be filled with a stored msg
 Description:
	it pops a message from the circular buffer
 Example:
	read(&CAN_TxMsg);
	if(CAN_TxMsg.id > 0)
	{
	}
 
 */
void CAN_read(msgCAN *message)
{
    
    if (_CAN_RX_BUFFER.head==_CAN_RX_BUFFER.tail)
    {
        //It means no data
        message->id=0;
        
    }else{
        *message=_CAN_RX_BUFFER.buffer[_CAN_RX_BUFFER.tail];
        _CAN_RX_BUFFER.tail=(_CAN_RX_BUFFER.tail+1)%RX_CAN_BUFFER_SIZE;
    }
    
}
// ----------------------------------------------------------------------------
