/* trmst main

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "driver/gpio.h"

#include "ssd1366.h"
#include "font8x8_basic.h"



/*
 * GPIO status:
 *	GPIO34: output (relay1)
 *	GPIO35: output (relay2)
 *	GPIO33:(clk - encoder) input
 *	GPIO25:(dt - encoder) input
 *	GPIO26:(sw - encoder) input
 *
 * Pin assignment:
 *
 * - slave :
 *    ds2482-100
 *    ssd1306 oled display
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Test items:
 */
 
 
#define GPIO_RELAY1    14
#define GPIO_RELAY2    12
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_RELAY1) | (1ULL<<GPIO_RELAY2))
#define GPIO_ENC_CLK     33
#define GPIO_ENC_DT		 25
#define GPIO_ENC_SW		 26
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_ENC_CLK) | (1ULL<<GPIO_ENC_DT) | (1ULL<<GPIO_ENC_SW)) 
#define ESP_INTR_FLAG_DEFAULT 0



#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */


#define I2C_EXAMPLE_SLAVE_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_EXAMPLE_SLAVE_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */

#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define tag "SSD1306"


#define ssd1306_ADDR                       0x3C             /*!< oled display slave address, you can set any 7bit value */




#define ReadROM 0x33	
#define SkipROM 0xCC
#define MatchROM 0x55
#define ConvertT 0x44



SemaphoreHandle_t print_mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;




static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}



static void ENC(void* arg)
{
    uint32_t io_num;
    for(;;) {
		xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);
		if(io_num == GPIO_ENC_CLK)
		{
			//vTaskDelay(10 / portTICK_RATE_MS);
			xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);
			if (io_num == GPIO_ENC_DT)
			{
				printf("clockwise rotation\n");
			}
				
		}	
		else if(io_num == GPIO_ENC_DT)
		{
			//vTaskDelay(10 / portTICK_RATE_MS);
			xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);
			if (io_num == GPIO_ENC_CLK)
			{
				printf("counter clockwise rotation\n");
			}
				
		}
		else if(io_num == GPIO_ENC_SW)
		{
			printf("Button is pressed\n");
		}
    }
}




//*********************DS2482 1-Wire Operations***********************************
//OWReset
//OWWriteBit/OWReadBit
//OWWriteByte
//OWReadByte
//OWBlock
//OWSearch/1-Wire Triplet Command
//--------------------------------------------------------------------------
// Reset all of the devices on the 1-Wire Net and return the result.
// S AD,0 [A] 1WRS [A] S AD,1 [A] [byte - STATUS register] A [byte STATUS register] A\ P
// Returns: TRUE(1):  presence pulse(s) detected, device(s) reset
//          FALSE(0): no presence pulses detected
//
int OWReset(void)
{
   uint8_t status = 0;
   uint8_t *st; // pointer to status
   uint8_t SD; //Short detected
   uint8_t PPD; //presence pulse detected
   int poll_count = 0;
   st = &status;
   
   
   
   
   //int poll_count = 0;
   //   S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                   \--------/
   //                       Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd); // S - start
   i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0 - [Acknowledged]
   i2c_master_write_byte(cmd, CMD_1WRS, ACK_CHECK_EN); //1WRS - [Acknowledged]
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   switch(ret){
			case ESP_OK:
				printf("[OWReset()] - CMD_1WRS = OK\n");
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[OWReset()] - CMD_1WRS Parameter error \n");
				return FALSE;
			case ESP_FAIL:
				printf("[OWReset()] - CMD_1WRS Sending command error, slave doesn't ACK the transfer \n");
				return FALSE;
			case ESP_ERR_INVALID_STATE:
				printf("[OWReset()] - CMD_1WRS I2C driver not installed or not in master mode \n");
				return FALSE;
			case ESP_ERR_TIMEOUT:
				printf("[OWReset()] - CMD_1WRS Operation timeout because the bus is busy \n");
				return FALSE;
			default:
				printf( "OWReset() hmmmmmmmmmmmmmmm\n" );
				return FALSE;
		}
   cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);// S - start
   i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1 - [Acknowledged]
	
   // loop checking 1WB bit for completion of 1-Wire operation
   // abort if poll limit reached
   do
   {
      i2c_master_read_byte(cmd, st, ACK_VAL); //[byte - STATUS register] [Acknowledged]
	  //printf("counter: %d\n", ++count);
   }
   while ((*st & STATUS_1WB) && (poll_count++ < POLL_LIMIT));//Repeat until 1WB bit has changed to 0 - getbits(tmp, 0, 1) - не проверяется!!!!!!!!!!!

   i2c_master_read_byte(cmd, st, NACK_VAL); //[byte - STATUS register] [NotAcknowledged]
   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   switch(ret){
			case ESP_OK:
				printf("[OWReset()] - *st = %d\n", *st);
				printf("[OWReset()] - status = %d\n", status);
				if(*st & STATUS_PPD)
				{
					PPD = TRUE;
					printf("[OWReset()] - PPD = %d\n", PPD);
				}
				else
				{
					PPD = FALSE;
					printf("[OWReset()] - PPD = %d\n", PPD);
				}
				
				if(*st & STATUS_SD)	
				{
					SD = TRUE;
					printf("[OWReset()] - SD = %d\n", SD);
				}
				else{
					SD = FALSE;
					printf("[OWReset()] - SD = %d\n", SD);
				}
				
				
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[OWReset()] - CMD_1WRS Parameter error \n");
				return FALSE;
			case ESP_FAIL:
				printf("[OWReset()] - CMD_1WRS Sending command error, slave doesn't ACK the transfer \n");
				return FALSE;
			case ESP_ERR_INVALID_STATE:
				printf("[OWReset()] - CMD_1WRS I2C driver not installed or not in master mode \n");
				return FALSE;
			case ESP_ERR_TIMEOUT:
				printf("[OWReset()] - CMD_1WRS Operation timeout because the bus is busy \n");
				return FALSE;
			default:
				printf( "OWReset() hmmmmmmmmmmmmmmm\n" );
				return FALSE;
		}
		
	
   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return FALSE;
   }

	
   // check for short condition
   
   // check for presence detect
   
   if (PPD)
      return TRUE;
   else
      return FALSE;
}




//----------------------------------------------------------------------------
// Send bit of communication to the 1-wire net and return the result
// 1 bit read from the 1-wire net. The parametr 'sendbit' least
// significant bit is used and the least significant bit of the
// result is the return bit.
// 'sendbit' - the least significant bit is the bit to send
// Returns: 0 - 0 bit read from sendbit
//			1 - 1 bit read from sendbit
uint8_t OWTouchBit(uint8_t sendbit)
{
	uint8_t status;
	uint8_t *st; // pointer to status
	st = &status;
	int poll_count = 0;
	// 1-wire bit (Case B)
	// S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
	// 										   \--------/
	//                      Repeat until 1WB bit has changed to 0
	// [] indicates from slave
	// BB indicates byte containing bit value in msbit
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0 - [Acknowledged]
	i2c_master_write_byte(cmd, CMD_1WSB, ACK_CHECK_EN); //1WSB - [Acknowledged]
	i2c_master_write_byte(cmd, sendbit ? 0x80 : 0x00, ACK_CHECK_EN); //1WSB - [Acknowledged]
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	switch(ret){
			case ESP_OK:
				printf("[OWTouchBit()] - CMD_1WSB = OK\n");
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[OWTouchBit()] - CMD_1WSB Parameter error \n");
				return FALSE;
			case ESP_FAIL:
				printf("[OWTouchBit()] - CMD_1WSB Sending command error, slave doesn't ACK the transfer \n");
				return FALSE;
			case ESP_ERR_INVALID_STATE:
				printf("[OWTouchBit()] - CMD_1WSB I2C driver not installed or not in master mode \n");
				return FALSE;
			case ESP_ERR_TIMEOUT:
				printf("[OWTouchBit()] - CMD_1WSB Operation timeout because the bus is busy \n");
				return FALSE;
			default:
				printf( "OWTouchBit() hmmmmmmmmmmmmmmm\n" );
				return FALSE;
		}
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);// S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1 - [Acknowledged]
	// loop checking 1WB bit for completion of 1-Wire operation
    // abort if poll limit reached
	// loop checking 1WB bit for completion of 1-Wire operation
   // abort if poll limit reached
    do
    {
      i2c_master_read_byte(cmd, st, ACK_VAL); //[byte - STATUS register] [Acknowledged]
	  //printf("counter: %d\n", ++count);
    }
    while ((*st & STATUS_1WB) && (poll_count++ < POLL_LIMIT));//Repeat until 1WB bit has changed to 0 - getbits(tmp, 0, 1) - не проверяется!!!!!!!!!!!

    i2c_master_read_byte(cmd, st, NACK_VAL); //[byte - STATUS register] [NotAcknowledged]
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    switch(ret){
			case ESP_OK:
				printf("[OWTouchBit()] - *st = %d\n", *st);
				printf("[OWTouchBit()] - status = %d\n", status);
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[OWTouchBit()] - CMD_1WRS Parameter error \n");
				return FALSE;
			case ESP_FAIL:
				printf("[OWTouchBit()] - CMD_1WRS Sending command error, slave doesn't ACK the transfer \n");
				return FALSE;
			case ESP_ERR_INVALID_STATE:
				printf("[OWTouchBit()] - CMD_1WRS I2C driver not installed or not in master mode \n");
				return FALSE;
			case ESP_ERR_TIMEOUT:
				printf("[OWTouchBit()] - CMD_1WRS Operation timeout because the bus is busy \n");
				return FALSE;
			default:
				printf( "OWTouchBit() hmmmmmmmmmmmmmmm\n" );
				return FALSE;
		}
	
	if(poll_count >= POLL_LIMIT)
	{
		DS2482_reset();
		return FALSE;
	}
	
	
    if(*st & STATUS_SBR)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	
	
}



//----------------------------------------------------------------------------
// Send 1 bit of communication to the 1-wire net.
// The parameter 'sendbit' least significant bit is used.
// 'sendbit' - 1 bit to send (least significant byte)

void OWWriteBit(uint8_t sendbit)
{
	OWTouchBit(sendbit);
}



//----------------------------------------------------------------------------
// Reads 1 bit of communication from the 1-wire net and returns the result
// Returns: 1 bit read from 1-wire net
uint8_t OWReadBit(void)
{
	return OWTouchBit(0x01);
}






//-------------------------------------------------------------------------------------
// Send 8 bits of communication to the 1-wire net and verify that the
// 8 bits read from the 1-wire net are the same (write operation).
// The parameter 'sendbyte' least significant 8 bits are used.
// 'sendbyte' - 8 bits to send (least significant byte)
// Returns: TRUE: bytes written and echo was the same
// 			FALSE: echo was not the same
void OWWriteByte(uint8_t sendbyte)
{
	uint8_t status;
	int poll_count = 0;
	uint8_t *st; // pointer to status
	st = &status;
	// 1-wire write byte (Case B)
	// S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
	// 										  \--------/
	//							Repeat until 1WB bit has changed to 0
	// DD data to write
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0 - [Acknowledged]
	i2c_master_write_byte(cmd, CMD_1WWB, ACK_CHECK_EN); //1WSB - [Acknowledged]
	i2c_master_write_byte(cmd, sendbyte, ACK_CHECK_EN); //DD - [Acknowledged]
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	switch(ret){
			case ESP_OK:
				printf("[OWWriteByte()] - CMD_1WWB = OK\n");
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[OWWriteByte()] - CMD_1WWB Parameter error \n");
			case ESP_FAIL:
				printf("[OWWriteByte()] - CMD_1WWB Sending command error, slave doesn't ACK the transfer \n");
			case ESP_ERR_INVALID_STATE:
				printf("[OWWriteByte()] - CMD_1WWB I2C driver not installed or not in master mode \n");
			case ESP_ERR_TIMEOUT:
				printf("[OWWriteByte()] - CMD_1WWB Operation timeout because the bus is busy \n");
			default:
				printf( "OWWriteByte() hmmmmmmmmmmmmmmm\n" );
		}
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);// S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1 - [Acknowledged]
	
	// loop checking 1WB bit for completion of 1-Wire operation
    // abort if poll limit reached
	do
    {
      i2c_master_read_byte(cmd, st, ACK_VAL); //[byte - STATUS register] [Acknowledged]
	  //printf("counter: %d\n", ++count);
    }
    while ((*st & STATUS_1WB) && (poll_count++ < POLL_LIMIT));//Repeat until 1WB bit has changed to 0 - getbits(tmp, 0, 1) - не проверяется!!!!!!!!!!!

    i2c_master_read_byte(cmd, st, NACK_VAL); //[byte - STATUS register] [NotAcknowledged]
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    switch(ret){
			case ESP_OK:
				printf("[OWWriteByte()] - *st = %d\n", *st);
				printf("[OWWriteByte()] - status = %d\n", status);
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[OWWriteByte()] - CMD_1WWB Parameter error \n");
			case ESP_FAIL:
				printf("[OWWriteByte()] - CMD_1WWB Sending command error, slave doesn't ACK the transfer \n");
			case ESP_ERR_INVALID_STATE:
				printf("[OWWriteByte()] - CMD_1WWB I2C driver not installed or not in master mode \n");
			case ESP_ERR_TIMEOUT:
				printf("[OWWriteByte()] - CMD_1WWB Operation timeout because the bus is busy \n");
			default:
				printf( "OWWriteByte() hmmmmmmmmmmmmmmm\n" );
		}
		
	if(poll_count >= POLL_LIMIT)
	{
		DS2482_reset();
	}
}


//-------------------------------------------------------------------------------------------------------------------
// Send 8 bits of read communication to the 1-wire net and return the 
// result 8 bits read from the 1-wire net.
//Returns: 8 bits read from 1-wire net
uint8_t OWReadByte(void)
{
	uint8_t status;
	uint8_t data;
	int poll_count = 0;
	uint8_t *st; // pointer to status
	st = &status;
	// 1-wire write byte (Case C)
	// S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\ Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
	// 								   \--------/
	//							Repeat until 1WB bit has changed to 0
	// DD - read data
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0 - [Acknowledged]
	i2c_master_write_byte(cmd, CMD_1WRB, ACK_CHECK_EN); //1WRB - [Acknowledged]
	
	i2c_master_start(cmd);// S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1 - [Acknowledged]
	
	do
    {
      i2c_master_read_byte(cmd, st, ACK_VAL); //[byte - STATUS register] [Acknowledged]
	  //printf("counter: %d\n", ++count);
    }
    while ((*st & STATUS_1WB) && (poll_count++ < POLL_LIMIT));//Repeat until 1WB bit has changed to 0 - getbits(tmp, 0, 1) - не проверяется!!!!!!!!!!!

    i2c_master_read_byte(cmd, st, NACK_VAL); //[byte - STATUS register] [NotAcknowledged]
	
	if(poll_count >= POLL_LIMIT)
	{
		DS2482_reset();
		return FALSE;
	}
	
	i2c_master_start(cmd); // S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0 - [Acknowledged]
	i2c_master_write_byte(cmd, CMD_SRP, ACK_CHECK_EN); //SRP - [Acknowledged]
	i2c_master_write_byte(cmd, 0xE1, ACK_CHECK_EN); //SRP - [Acknowledged]
	
	
	i2c_master_start(cmd);// S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1 - [Acknowledged]
	i2c_master_read_byte(cmd, &data, NACK_VAL); //[DD] notAcknowledged
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	switch(ret){
			case ESP_OK:
				printf("[OWReadByte()] - data = %d\n", data);
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[OWReadByte()] -  Parameter error \n");
			case ESP_FAIL:
				printf("[OWReadByte()] -  Sending command error, slave doesn't ACK the transfer \n");
			case ESP_ERR_INVALID_STATE:
				printf("[OWReadByte()] - I2C driver not installed or not in master mode \n");
			case ESP_ERR_TIMEOUT:
				printf("[OWReadByte()] - Operation timeout because the bus is busy \n");
			default:
				printf( "OWReadByte() hmmmmmmmmmmmmmmm\n" );
		}
	return data;
}

//----------------------------------------------------------------------------------------------------------
// Send 8 bits of communication to the 1-wire net and return the
// result 8 bits read from the 1-wire net. The parameter 'sendbyte'
// least significant 8 bits are used and the least significant 8 bits
// of the result are the return byte.
// 'sendbyte' - 8 bits send (least significant byte)
// Returns: 8 bits read from sendbyte
uint8_t OWTouchByte(uint8_t sendbyte)
{
	if (sendbyte == 0xff)
		return OWReadByte();
	else
	{
		OWWriteByte(sendbyte);
		return sendbyte;
	}
} 


//----------------------------------------------------------------------------------------------------------
// The 'OWBlock' transfers a block of data to and from the 
// 1-wire net. The result is returned in the same buffer.
// 'tran_buf' - pointer to a block of uint8_t of length 'tran_len'
//				that will be sent to the 1-wire net.
// 'tran_len' - length in bytes to transfer
void OWBlock(uint8_t *tran_buf, int tran_len)
{
	int i;
	for (i=0; i < tran_len; i++)
		tran_buf[i] = OWTouchByte(tran_buf[i]);
}


//----------------------------------------------------------------------------------------------------------
// Use the DS2482 help command '1-wire triplet' to perform one bit of a
// 1-wire search.
// This command does two read bits and one writte bit. The write bit
// is either the default direction (all device have same bit) or in case of 
// a discrepancy, the 'search_direction' parameter is used.
// Returns - The DS2482 status byte result from the triplet command
uint8_t DS2482_search_triplet(uint8_t search_direction)
{
	uint8_t status;
	int poll_count = 0;
	uint8_t *st; // pointer to status
	st = &status;
	
	// 1-wire triplet (Case B)
	// S AD,0 [A] 1WT [A] SS [A] Sr [A] AD,1 [A] [Staus] A [Status] A\ P
	// 											 \--------/
	//								 Repeat until 1WB bit has changed to 0
	//SS indicates byte containing search direction bit value in msbit
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0 - [Acknowledged]
	i2c_master_write_byte(cmd, CMD_1WT, ACK_CHECK_EN); //1WT - [Acknowledged]
	i2c_master_write_byte(cmd, search_direction ? 0x80 : 0x00, ACK_CHECK_EN); //SS - [Acknowledged]
	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	switch(ret){
			case ESP_OK:
				printf("[DS2482_search_triplet()] - CMD_1WT = OK\n");
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[DS2482_search_triplet()] - CMD_1WT Parameter error \n");
			case ESP_FAIL:
				printf("[DS2482_search_triplet()] - CMD_1WT Sending command error, slave doesn't ACK the transfer \n");
			case ESP_ERR_INVALID_STATE:
				printf("[DS2482_search_triplet()] - CMD_1WT I2C driver not installed or not in master mode \n");
			case ESP_ERR_TIMEOUT:
				printf("[DS2482_search_triplet()] - CMD_1WT Operation timeout because the bus is busy \n");
			default:
				printf( "DS2482_search_triplet() hmmmmmmmmmmmmmmm\n" );
		}
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);// S - start
    i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1 - [Acknowledged]
	do
    {
      i2c_master_read_byte(cmd, st, ACK_VAL); //[byte - STATUS register] [Acknowledged]
	  //printf("counter: %d\n", ++count);
    }
    while ((*st & STATUS_1WB) && (poll_count++ < POLL_LIMIT));//Repeat until 1WB bit has changed to 0 - getbits(tmp, 0, 1) - не проверяется!!!!!!!!!!!
	i2c_master_read_byte(cmd, st, NACK_VAL); //[byte - STATUS register] [NotAcknowledged]
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    switch(ret){
			case ESP_OK:
				printf("[DS2482_search_triplet()] - *st = %d\n", *st);
				printf("[DS2482_search_triplet()] - status = %d\n", status);
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[DS2482_search_triplet()] - Parameter error \n");
			case ESP_FAIL:
				printf("[DS2482_search_triplet()] - Sending command error, slave doesn't ACK the transfer \n");
			case ESP_ERR_INVALID_STATE:
				printf("[DS2482_search_triplet()] - I2C driver not installed or not in master mode \n");
			case ESP_ERR_TIMEOUT:
				printf("[DS2482_search_triplet()] - Operation timeout because the bus is busy \n");
			default:
				printf( "DS2482_search_triplet() hmmmmmmmmmmmmmmm\n" );
		}
		
	if(poll_count >= POLL_LIMIT)
	{
		DS2482_reset();
		return FALSE;
	}
	
	return status;
}





// TEST BUILD
static unsigned char crc_table[] = {
        0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
      157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
       35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
      190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
       70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
      219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
      101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
      248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
      140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
       17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
      175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
       50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
      202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
       87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
      233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
      116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//----------------------------------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current 
// global 'crc8' value. 
// Returns current global crc8 value
uint8_t calc_crc8(uint8_t value)
{
	// See Application Note 27
   
	// TEST BUILD
   crc8 = crc_table[crc8 ^ value];
   return crc8;
}










//----------------------------------------------------------------------------------------------------------
// The 'OWSearch' function does a general search. This function
// continues from the previus search state. The search state can be 
// reset by using the 'OWFirst' function.
// This function contains one parameter 'alarm_only'.
// When 'alarm_only' is TRUE (1) the find alarm command 0xEC
// is sent instead of the normal search command 0xF0.
// Using the find alarm command 0xEC will limit the search to only 
// 1-wire devices that are in an 'alarm' state.
// Return: TRUE(1): when a 1-wire device was found and its
//					serial number placed in the global ROM
//			FALSE(0): when no new device was found. Either the 
//						last search was the last device or there 
//						are no devices on the 1-wire net.
uint8_t OWSearch()
{
	uint8_t id_bit;
	uint8_t cmp_id_bit;
	
	uint8_t search_direction;
	uint8_t status;
	
	// initialize for search
	uint8_t id_bit_number = 1;
	uint8_t last_zero = 0;
	uint8_t rom_byte_number = 0;
	uint8_t search_result = FALSE; // returned
	uint8_t rom_byte_mask = 1;
	
	crc8 = 0;
	
	
	
	// if the last call was not the last one 
	if (!LastDeviceFlag)
	{
		// 1-wire reset
		if (!OWReset())
		{
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = FALSE;
			LastFamilyDiscrepancy = 0;
			return FALSE;
			
		}
		
		// issue the search command
		// выдать команду поиска
		OWWriteByte(SearchROM); // 0xF0
		
		// loop to do the search
		// Цикл поиска
		do
		{
			if(id_bit_number < LastDiscrepancy)
			{
				if((ROM_NO[rom_byte_number] & rom_byte_mask) > 0)
					search_direction = 1;
				else
					search_direction = 0;
			}
			else
			{
				// if equal to last pick 1, if not then pick 0
				if(id_bit_number == LastDiscrepancy)
					search_direction = 1;
				else
					search_direction = 0;
			}
			
			
			// Perform a triple operation on the ds2482 which will perform
			// 2 read bits and 1 write bit
			status = DS2482_search_triplet(search_direction);
			
			//check bit results in status byte
			id_bit = ((status & STATUS_SBR) == STATUS_SBR);
			cmp_id_bit = ((status & STATUS_TSB) == STATUS_TSB);
			search_direction = ((status & STATUS_DIR) == STATUS_DIR) ? 1 : 0;
			
			// check for no devices on 1-wire
			if((id_bit) && (cmp_id_bit))
				break;
			else
			{
				if((!id_bit) && (!cmp_id_bit) && (search_direction == 0))
				{
					last_zero = id_bit_number;
					
					// check for last discrepancy in family
					if(last_zero < 9)
						LastFamilyDiscrepancy = last_zero;
				}
				
				// set or clear  the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if(search_direction == 1)
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;
				
				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;
				
				// if the mask is 0 then go to new SerialNum byte rom_byte_number
				// and reset mask
				if (rom_byte_mask == 0)
				{
					calc_crc8(ROM_NO[rom_byte_number]); // accumulate the CRC
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		}
		while(rom_byte_number < 8);  // loop until through all ROM byte 0-7
		
		// if the search was successful then
		if(!((id_bit_number < 65) || (crc8 != 0)))
		{
			// search successful so set LastDiscrepancy, LastDeviceFlag
			// search_result
			LastDiscrepancy = last_zero;
			
			//check for last device
			if(LastDiscrepancy == 0)
				LastDeviceFlag = TRUE;
			
			search_result = TRUE;
		}	
	}
	
	// if no device found then reset counters so next
	// 'search' will be like a first
	
	if(!search_result || (ROM_NO[0] == 0))
	{
		LastDiscrepancy = 0;
		LastDeviceFlag = FALSE;
		LastFamilyDiscrepancy = 0;
		search_result = FALSE;
	}
	
	return search_result;
} 




//-------------------------------------------------------------------------------------------------------------
// Find the 'first' devices on the 1-wire network
// Return TRUE: device found, ROM number in ROM_NO buffer
//		  FALSE: no device present
uint8_t OWFirst()
{
	// reset the serach state
	LastDiscrepancy = 0;
	LastDeviceFlag = FALSE;
	LastFamilyDiscrepancy = 0;
	
	return OWSearch();
}


//-----------------------------------------------------------------------------------------------------------
// Find the 'next' devices on the 1-wire network
// Return TRUE: device found, ROM number in ROM_NO buffer
//		  FALSE: device not found, end of search
uint8_t OWNext()
{
	//leave the search state alone
	return OWSearch();
}




/*
 * @brief i2c master initialization
 */
static void i2c_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}






static void ds2482_task(void* arg)
{
    
    /*
    uint32_t task_idx = (uint32_t) arg;
    uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_wr = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_rd = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t sensor_data_h, sensor_data_l;
	*/
    int cnt = 0;
	int count;
	DS2482_detect();
	vTaskDelay(1000 / portTICK_RATE_MS);
	DS2482_detect();
	
	// find ALL devices
    printf("\nFIND ALL\n");
    count = 0;
	int i;
    uint8_t rslt = OWFirst();
	while (rslt)
	{
		// print device found
		for (i = 7; i >= 0; i--)
			printf("%02X", ROM_NO[i]);
		printf("  %d\n", ++count);

		rslt = OWNext();
	}
	
	
	
    while (1) {
        printf("test cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_RATE_MS);
		if(OWReset())
		{
			printf("1 wire device detected\n");
		}
		else
		{
			printf("1 wire device not detected\n");
		}
		vTaskDelay(3000 / portTICK_RATE_MS);
    }
}





void app_main()
{
	
	
	
	
	gpio_config_t io_conf;
	//Настройки GPIO для релейного ВЫХОДа
    //disable interrupt - отключитли прерывания
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	
    //set as output mode - установка в режим Выход
    io_conf.mode = GPIO_MODE_OUTPUT;
	
    //bit mask of the pins that you want to set,e.g.GPIO14/12
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	
    //disable pull-down mode - отключитли подтяжку к земле
    io_conf.pull_down_en = 0;
	
    //disable pull-up mode - отключили подтяжку к питанию
    io_conf.pull_up_en = 0;
	
    //configure GPIO with the given settings - конфигурирование портов с данными настройками (Выход)
    gpio_config(&io_conf);
	
	//Настройки GPIO Энкодера на ВХОД
	//interrupt of falling edge - прерывание по спадающему фронту и по возрастающему фронту
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO33/25/26 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    
	//enable pull-up mode
    //io_conf.pull_up_en = 1;
	
    gpio_config(&io_conf);
	
	//change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_ENC_SW, GPIO_INTR_NEGEDGE); //Button pressed in gpio26
	
	
	//create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(ENC, "ENC", 2048, NULL, 10, NULL);
	
	//install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_ENC_CLK, gpio_isr_handler, (void*) GPIO_ENC_CLK);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_ENC_DT, gpio_isr_handler, (void*) GPIO_ENC_DT);
	//hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_ENC_SW, gpio_isr_handler, (void*) GPIO_ENC_SW);
	
	
	
	
    //print_mux = xSemaphoreCreateMutex();
    
    i2c_master_init();
    xTaskCreate(ds2482_task, "ds2482_task", 1024 * 2, (void* ) 1, 10, NULL);

}

