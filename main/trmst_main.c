/* trmst main

  
*/
#include <stdio.h>

#include "driver/gpio.h"
#include "ds2482.h"
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




#define tag "SSD1306"


#define ssd1306_ADDR                       0x3C             /*!< oled display slave address, you can set any 7bit value */


#define ReadROM 0x33	
#define SkipROM 0xCC
#define MatchROM 0x55
#define ConvertT 0x44



SemaphoreHandle_t print_mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
portBASE_TYPE xStatusReadTemp; // status task create
xTaskHandle xRead_Temp_Handle; // identification Read Temp task


	

extern uint8_t short_detected; //short detected on 1-wire net
extern uint8_t crc8;
extern uint8_t crc_tbl[];








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





/*
 * @brief i2c master initialization
 */
static void i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}







static void vReadTemp(void* arg)
{
	/* Find Devices */
	uint8_t rslt = 0;
	uint8_t LastDiscrepancy = 0;
	uint8_t LastFamilyDiscrepancy = 0; 
	uint8_t LastDeviceFlag = 0;
	
	uint8_t *pROM_NO[4];
	int j, i = 0;
	int count = 0;
	vTaskDelay(3000 / portTICK_RATE_MS);
	if(DS2482_detect())
	{
		/*ds2482 i2c/1-wire bridge detected*/
		if(OWReset())
		{
			/*1-wire device detected*/
			// find address ALL devices
			printf("\nFIND ALL ******** \n");
			do{
				pROM_NO[i] = (uint8_t*) malloc(8); //memory for address
				rslt = OWSearch(&LastDiscrepancy, &LastFamilyDiscrepancy, &LastDeviceFlag, pROM_NO[i]);
				if(rslt)
				{
					count++;
					for(j = 7; j >= 0; j--)
					{
						printf("%02X", *(pROM_NO[i] + j));
					}
					printf("\n count = %d\n", count);
				}
				else
					printf("1-wire device not find\n");
				i++;
			}
			while(i <= 4 && rslt);
		}
		else
			printf("1-wire device not detected\n");
		
	}
	else
		printf("ds2482 i2c/1-wire bridge not detected\n");
	
	while(1)
	{
		
		
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
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
	
	//change gpio interrupt type for one pin
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
	/*
	rom_no_queue = xQueueCreate(8, sizeof(uint8_t));
	if (rom_no_queue != NULL)
	{
		xStatusFindDevices = xTaskCreate(vFindDevices, "vFindDevices", 1024 * 2, NULL, 10, &xFindDev_Handle);
		if(xStatusFindDevices == pdPASS)
			printf("Task vFindDevices is created!\n");
		else
			printf("Task vFindDevices is not created\n");
	
		xStatusReadTemp = xTaskCreate(vReadTemp, "vReadTemp", 1024 * 2, NULL, 10, &xRead_Temp_Handle);
		if(xStatusReadTemp == pdPASS)
			printf("Task vReadTemp is created!\n");
		else
			printf("Task vReadTemp is not created\n");
	}
	
	*/
	
	xStatusReadTemp = xTaskCreate(vReadTemp, "vReadTemp", 1024 * 2, NULL, 10, &xRead_Temp_Handle);
		if(xStatusReadTemp == pdPASS)
			printf("Task vReadTemp is created!\n");
		else
			printf("Task vReadTemp is not created\n");
	


}

