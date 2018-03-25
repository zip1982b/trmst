/* trmst main

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/i2c.h"
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

#define DS2482_ADDR			               0x18             /*!< slave address for DS2482 bridge */
#define ssd1306_ADDR                       0x3C             /*!< oled display slave address, you can set any 7bit value */



#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */


#define CONFIG_APU 0x01
#define CMD_DRST 0xF0	//command 'device reset'
#define CMD_WCFG 0xD2	//Command 'Write Configuration'
#define CMD_1WRS 0xB4	//command '1 wire reset'



#define TRUE 1
#define FALSE 0

// DS2482 state
unsigned char I2C_address;
int c1WS;// 1-Wire speed
int cSPU;// Strong pullup
int cPPM;// Presence pulse masking
int cAPU;// Active pullup
int short_detected;







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



//-------------------------DS2482-100------------------------------------
// Perform a device reset on the DS2482
//
// Returns: TRUE if device was reset
//          FALSE device not detected or failure to perform reset
//
static int DS2482_reset()
{
    // Device Reset
	// S - start
	// AD,0 - select DS2482 for Write Access
	// [A] - Acknowledged
	// DRST - Command 'Device Reset', F0h
	// [A] - Acknowledged
	// S - start
	// AD,1 - select DS2482 for Read Access
	// [A] - Acknowledged
	// [SS] - SS status byte to read to verify state
	// 	A\ - not Acknowledged
	// P - STOP Condition
    //   S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
    //  [] indicates from slave
    //  SS status byte to read to verify state
	
	uint8_t status;
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd); // S - start
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0 - [Acknowledged]
	i2c_master_write_byte(cmd, CMD_DRST, ACK_CHECK_EN); //DRST - [Acknowledged]
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	
	switch(ret){
			case ESP_OK:
				printf("[DS2482_reset()] - CMD_DRST = OK \n");
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[DS2482_reset()] - Parameter error \n");
				return FALSE;
			case ESP_FAIL:
				printf("[DS2482_reset()] - Sending command error, slave doesn't ACK the transfer \n");
				return FALSE;
			case ESP_ERR_INVALID_STATE:
				printf("[DS2482_reset()] - I2C driver not installed or not in master mode \n");
				return FALSE;
			case ESP_ERR_TIMEOUT:
				printf("[DS2482_reset()] - Operation timeout because the bus is busy \n");
				return FALSE;
			default:
				printf( "hmmmmmmmmmmmmmmm\n" );
				return FALSE;
		}
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);// S - start
		i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN);//AD,1 - [Acknowledged]
		i2c_master_read_byte(cmd, &status, NACK_VAL); //[SS] notAcknowledged
		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd);
		switch(ret){
			case ESP_OK:
				printf("[DS2482_reset()] - status = %d\n", status);
				// check for failure due to incorrect read back of status
				return ((status & 0xF7) == 0x10); //return TRUE;
			case ESP_ERR_INVALID_ARG:
				printf("[DS2482_reset()] - Parameter error \n");
				return FALSE;
			case ESP_FAIL:
				printf("[DS2482_reset()] - Sending command error, slave doesn't ACK the transfer \n");
				return FALSE;
			case ESP_ERR_INVALID_STATE:
				printf("[DS2482_reset()] - I2C driver not installed or not in master mode \n");
				return FALSE;
			case ESP_ERR_TIMEOUT:
				printf("[DS2482_reset()] - Operation timeout because the bus is busy \n");
				return FALSE;
			default:
				printf( "hmmmmmmmmmmmmmmm\n" );
				return FALSE;
		}
}


//--------------------------------------------------------------------------
// Write the configuration register in the DS2482. The configuration
// options are provided in the lower nibble of the provided config byte.
// The uppper nibble in bitwise inverted when written to the DS2482.
//
// Returns:  TRUE: config written and response correct
//           FALSE: response incorrect
//
static int DS2482_write_config(uint8_t config)
{
   // Write configuration (Case A)
   //   S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
   //  [] indicates from slave
   //  CF configuration byte to write
   
   uint8_t read_config;
   uint8_t tmp;
   uint8_t reg_config;
   tmp = config;
   reg_config = config | (~tmp << 4);//is the one’s complement of the lower nibble
   printf("reg_config = %d\n", reg_config);
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd); // S - start
   i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0 - [Acknowledged]
   i2c_master_write_byte(cmd, CMD_WCFG, ACK_CHECK_EN); //WCFG - [Acknowledged]
   i2c_master_write_byte(cmd, reg_config, ACK_CHECK_EN); //CF - [Acknowledged]
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   switch(ret){
			case ESP_OK:
				printf("[DS2482_write_config()] - CMD_WCFG = OK\n");
				break;
			case ESP_ERR_INVALID_ARG:
				printf("[DS2482_write_config()] - CMD_WCFG Parameter error \n");
				return FALSE;
			case ESP_FAIL:
				printf("[DS2482_write_config()] - CMD_WCFG Sending command error, slave doesn't ACK the transfer \n");
				return FALSE;
			case ESP_ERR_INVALID_STATE:
				printf("[DS2482_write_config()] - CMD_WCFG I2C driver not installed or not in master mode \n");
				return FALSE;
			case ESP_ERR_TIMEOUT:
				printf("[DS2482_write_config()] - CMD_WCFG Operation timeout because the bus is busy \n");
				return FALSE;
			default:
				printf( "CMD_WCFG hmmmmmmmmmmmmmmm\n" );
				return FALSE;
		}
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);// S - start
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1 - [Acknowledged]
	i2c_master_read_byte(cmd, &read_config, NACK_VAL); //[SS] notAcknowledged
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	switch(ret){
			case ESP_OK:
				printf("[DS2482_write_config()] - CMD_WCFG = OK, read_config = %d\n", read_config);
				return TRUE;
			case ESP_ERR_INVALID_ARG:
				printf("[DS2482_write_config()] - CMD_WCFG Parameter error \n");
				return FALSE;
			case ESP_FAIL:
				printf("[DS2482_write_config()] - CMD_WCFG Sending command error, slave doesn't ACK the transfer \n");
				return FALSE;
			case ESP_ERR_INVALID_STATE:
				printf("[DS2482_write_config()] - CMD_WCFG I2C driver not installed or not in master mode \n");
				return FALSE;
			case ESP_ERR_TIMEOUT:
				printf("[DS2482_write_config()] - CMD_WCFG Operation timeout because the bus is busy \n");
				return FALSE;
			default:
				printf( "CMD_WCFG hmmmmmmmmmmmmmmm\n" );
				return FALSE;
		}
}


//--------------------------------------------------------------------------
// DS2428 Detect routine that sets the I2C address and then performs a
// device reset followed by writing the configuration byte to default values:
//   1-Wire speed (c1WS) = standard (0)
//   Strong pullup (cSPU) = off (0)
//   Presence pulse masking (cPPM) = off (0)
//   Active pullup (cAPU) = on (CONFIG_APU = 0x01)
//
// Returns: TRUE if device was detected and written
//          FALSE device not detected or failure to write configuration byte
//
int DS2482_detect()
{
   if (!DS2482_reset())
   {
	   printf("[DS2482_detect()] - ds2482-100 not detected or failure to perform reset\n");
	   return FALSE; // выход из функции
   }
   else
   {
	   printf("[DS2482_detect()] - ds2482-100 was reset\n");
	   //return TRUE; // выход из функции
   }

   // default configuration
   c1WS = FALSE;
   cSPU = FALSE;
   cPPM = FALSE;
   cAPU = CONFIG_APU;
   // write the default configuration setup
   //|~1WS = 1|~SPU = 1|1|~APU = 0|1WS = 0|SPU = 0|0|APU = 1| - Configuration Register DS2482-100 = 225(d), E1(h)
   //When read the upper nibble is always 0h.
   //When write register is the one’s complement of the lower nibble
   if (!DS2482_write_config(c1WS | cSPU | cPPM | cAPU))
   {
	   printf("[DS2482_detect()] - ds2482-100 failure to write configuration byte\n");
	   return FALSE;
   }
   else
   {
		printf("[DS2482_detect()] - ds2482-100 was written\n");
   }
   
   return TRUE;
   
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
   uint8_t status;
   int poll_count = 0;
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
      i2c_master_read_byte(cmd, &status, ACK_CHECK_EN); //[byte - STATUS register] [Acknowledged]
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));// ????????

   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
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

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return FALSE;
   }

   // check for short condition
   if (status & STATUS_SD)
      short_detected = TRUE;
   else
      short_detected = FALSE;

   // check for presence detect
   if (status & STATUS_PPD)
      return TRUE;
   else
      return FALSE;
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
	DS2482_detect();
    while (1) {
        printf("test cnt: %d\n", cnt++);
        DS2482_detect();
        vTaskDelay(1000 / portTICK_RATE_MS);
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

