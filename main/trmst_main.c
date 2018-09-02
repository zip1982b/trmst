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


#define ssd1306_ADDR 0x3C             /*!< oled display slave address, you can set any 7bit value */






SemaphoreHandle_t print_mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
portBASE_TYPE xStatusReadTemp; // status task create
xTaskHandle xRead_Temp_Handle; // identification Read Temp task


	

extern uint8_t short_detected; //short detected on 1-wire net
extern uint8_t crc8;
extern uint8_t crc_tbl[];
extern uint8_t ROM_NO[8];







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
	static uint8_t LastDiscrepancy = 0;
	static uint8_t LastFamilyDiscrepancy = 0; 
	static uint8_t LastDeviceFlag = 0;
	
	uint8_t *pROM_NO[3]; // 4 address = pROM_NO[0], pROM_NO[1], pROM_NO[2], pROM_NO[3].
	
	uint8_t i = 0;
	int j;
	int k;
	int n;
	int l;
	
	uint8_t sensors = 0;
	
	uint8_t get[9]; //get scratch pad
	int temp;
	float temperatura;
	
	

	vTaskDelay(1000 / portTICK_RATE_MS);
	if(DS2482_detect())
	{
		/*ds2482 i2c/1-wire bridge detected*/
		if(OWReset() && !short_detected)
		{
			/*1-wire device detected*/
			// find address ALL devices
			printf("\nFIND ALL ******** \n");
			do{
				//printf("i = %d\n", i);
				pROM_NO[i] = (uint8_t*) malloc(8); //memory for address
				//printf("address memory pROM_NO = %p\n", pROM_NO[i]);
				//printf("LastDiscrepancy value = %d\n", LastDiscrepancy);
				//printf("LastFamilyDiscrepancy value = %d\n", LastFamilyDiscrepancy);
				//printf("LastDeviceFlag value = %d\n", LastDeviceFlag);
				rslt = OWSearch(&LastDiscrepancy, &LastFamilyDiscrepancy, &LastDeviceFlag); //pROM_NO[i]
				if(rslt)
				{
					sensors++;
					for(j = 7; j >= 0; j--)
					{
						*(pROM_NO[i] + j) = ROM_NO[j];
						printf("%02X", *(pROM_NO[i] + j));
					}
					printf("\nSensor# %d\n", i + 1);
				}
				else{
					printf("1-wire device end find\n");
					free(pROM_NO[i]);
					printf("sensors = %d\n", sensors);
				}
				i++;
				//vTaskDelay(250 / portTICK_RATE_MS);
			}
			while(i <= 3 && rslt); // maximum 4 address
			printf("1-wire device end find\n");
		}
		else
			printf("1-wire device not detected (1) or short_detected = %d\n", short_detected);
		
		if(OWReset() && !short_detected){
			vTaskDelay(100 / portTICK_RATE_MS);
			OWWriteByte(SkipROM); //0xCC
			printf("SkipROM\n");
			OWWriteByte(WriteScratchpad); //0x4E
			printf("WriteScratchpad\n");
			OWWriteByte(0x4B); //TH
			printf("TH = 0x4B\n");
			OWWriteByte(0x46); //TL
			printf("TL = 0x46\n");
			OWWriteByte(0x7F); //Config register
			printf("Config = 0x7F - 12 bit\n");
		}
		else
			printf("1-wire device not detected (2) or short_detected = %d\n", short_detected);
		
	}
	else
		printf("ds2482 i2c/1-wire bridge not detected\n");
	
	
	while(1)
	{
		vTaskDelay(5000 / portTICK_RATE_MS);
		printf("**********************Cycle**********************************\n");
		if(OWReset() && !short_detected && sensors > 0)
		{
			vTaskDelay(100 / portTICK_RATE_MS);
			OWWriteByte(SkipROM); //0xCC - пропуск проверки адресов
			printf("SkipROM\n");
			OWWriteByte(ConvertT); //0x44 - все датчики измеряют свою температуру
			printf("ConvertT\n");
			
			
			for(l = 0; l < sensors; l++)
			{
				crc8 = 0;
				if(OWReset() && !short_detected)
				{
					vTaskDelay(100 / portTICK_RATE_MS);
					OWWriteByte(MatchROM); //0x55 - соответствие адреса
					printf("send MatchROM command\n");
					// send ROM address = 64 bit
					for(k = 0; k <= 7; k++)
					{
						//printf(" %X ", *(pROM_NO[l] + k));
						OWWriteByte(*(pROM_NO[l] + k));
					}
					OWWriteByte(ReadScratchpad); //0xBE
					printf("\n send ReadScratchpad command \n");
					for (n=0; n<9; n++)
					{
						get[n] = OWReadByte();
						//printf("get[%d] = %X\n", n, get[n]);
								
						//get[8] не надо проверять crc
						if(n < 8)
						{
							calc_crc8(get[n]); // accumulate the CRC
							//printf("crc8 = %X\n", crc8);
						}
						else if(get[8] == crc8)
							printf("crc = OK\n");
						else
						{
							printf("crc = NOK\n");
						}
					}
					printf("ScratchPAD data = %X %X %X %X %X %X %X %X %X\n", get[8], get[7], get[6], get[5], get[4], get[3], get[2], get[1], get[0]);
					// расчёт температуры
					//temp_msb = get[1]; 
					//temp_lsb = get[0];
					
					// -
					if(getbits(get[1], 7, 1))
					{
						temp = get[1] << 8 | get[0];
						temp = (~temp) + 1;
						temperatura = (temp * 0.0625) * (-1);
						printf("temp = %f *C\n", temperatura);
					}
					// +
					else 
					{
						temp = get[1] << 8 | get[0];
						temperatura = temp * 0.0625;
						printf("temp = %f *C\n", temperatura);
					}
						
				}
				else
					printf("1-wire device not detected(2) or short_detected = %d\n", short_detected);
			}
		}
		else
			printf("1-wire device not detected(1) or sensors = 0 or short_detected = %d\n", short_detected);
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

