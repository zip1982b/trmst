#include "ds2482.h"

/* DS2482 state */
uint8_t c1WS; //1-wire speed
uint8_t cSPU; //Strong pullup
uint8_t cPPM; //Precense pulse masking
uint8_t cAPU; //Active pullup
uint8_t short_detected; //short detected on 1-wire net


/* Search state */
uint8_t ROM_NO[8];
uint8_t LastDiscrepancy;	//Последнее не соответствие
uint8_t LastFamilyDiscrepancy; 
uint8_t LastDeviceFlag;
uint8_t crc8;


/* getbits: получает n бит,  начиная с p позиции*/
uint8_t getbits(uint8_t x, int p, int n)
{
	return (x >> (p+1-n)) & ~(~0 << n);
}




/* i2c command *
* S - start
* AD,0 - select ds2482 for write access
* [A] - Acknowledged
* DRST - Command "device reset" F0h
* WCFG - Command "write configuration" D2h
* SRP - Command "set read pointer" E1h
* 1WRS - Command "1-wire reset" B4h
* 1WWB - Command "1-wire write byte" A5h
* 1WRB - Command "1-wire read byte" 96h
* 1WT - Command "1-wire tripled" 78h
* 1WSB - Command "1-wire single bit" 87h
* AD,1 - select ds2482 for read access
* [SS] - status byte to read to verify state
* A\ - not Acknowledged
* P - stop condition
* [] indicates from slave
*/



uint8_t DS2482_reset(void)
{
	/*
	 *S AD,0 [A] DRST [A] P S AD,1 [A] [SS] A\ P 
	*/
	uint8_t status;
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd); //S
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0  - [A]
	i2c_master_write_byte(cmd, CMD_DRST, ACK_CHECK_EN); //DRST - [A]
	i2c_master_stop(cmd); // P
	esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	switch(ret){
		case ESP_OK:
			printf("[DS2482_reset()] - CMD_DRST = OK \n");
			break;
		case ESP_ERR_INVALID_ARG:
			printf("[DS2482_reset()] - Parameter error \n");
			return 0;
		case ESP_FAIL:
			printf("[DS2482_reset()] - Sending command error, slave doesn`t ACK the transfer \n");
			return 0;
		case ESP_ERR_INVALID_STATE:
			printf("[DS2482_reset()] - i2c driver not installed or not in master mode \n");
			return 0;
		case ESP_ERR_TIMEOUT:
			printf("[DS2482_reset()] - Operation timeout because the bus is busy \n");
			return 0;
		default:
			printf("[DS2482_reset()] - default block");
			return 0;
	}
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd); //S
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1  - [A]
	i2c_master_read_byte(cmd, &status, NACK_VAL); // [SS] - notA
	i2c_master_stop(cmd); // P
	ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	switch(ret){
		case ESP_OK:
			printf("[DS2482_reset()] - status = %d \n", status);
			/* check for failure due to incorrect read back of status */
			return ((status & 0xF7) == 0x10); //return 1
		case ESP_ERR_INVALID_ARG:
			printf("[DS2482_reset()] - Parameter error \n");
			return 0;
		case ESP_FAIL:
			printf("[DS2482_reset()] - Sending command error, slave doesn`t ACK the transfer \n");
			return 0;
		case ESP_ERR_INVALID_STATE:
			printf("[DS2482_reset()] - i2c driver not installed or not in master mode \n");
			return 0;
		case ESP_ERR_TIMEOUT:
			printf("[DS2482_reset()] - Operation timeout because the bus is busy \n");
			return 0;
		default:
			printf("[DS2482_reset()] - default block");
			return 0;
	}
}






uint8_t DS2482_write_config(uint8_t config)
{
	/*
	 * S AD,0 [A] WCFG [A] CF [A] P S AD,1 [A] [CF] A\ P
	*/
	uint8_t read_config;
	uint8_t tmp;
	uint8_t reg_config;
	tmp = config;
	reg_config = config | (~tmp << 4);//is the one`s complement of the lower nibble
	tmp = 0;
	printf("reg_config = %d \n", reg_config);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);  //S
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0  - [A]);
	i2c_master_write_byte(cmd, CMD_WCFG, ACK_CHECK_EN); //WCFG - [A]
	i2c_master_write_byte(cmd, reg_config, ACK_CHECK_EN); //CF - [A]
	i2c_master_stop(cmd); // P
	esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	switch(ret){
		case ESP_OK:
			printf("[DS2482_write_config()] - WCFG and CF = OK \n");
			break;
		case ESP_ERR_INVALID_ARG:
			printf("[DS2482_write_config()] - Parameter error (1) \n");
			return 0;
		case ESP_FAIL:
			printf("[DS2482_write_config()] - Sending command error, slave doesn`t ACK the transfer \n");
			return 0;
		case ESP_ERR_INVALID_STATE:
			printf("[DS2482_write_config()] - i2c driver not installed or not in master mode \n");
			return 0;
		case ESP_ERR_TIMEOUT:
			printf("[DS2482_write_config()] - Operation timeout because the bus is busy \n");
			return 0;
		default:
			printf("[DS2482_write_config()] - default block");
			return 0;
	}
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd); //S
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1  - [A]
	i2c_master_read_byte(cmd, &read_config, NACK_VAL); // [CF] - notA
	i2c_master_stop(cmd); // P
	ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	switch(ret){
		case ESP_OK:
			printf("[DS2482_write_config()] - read_config = %d \n", read_config);
			tmp = getbits(reg_config, 3, 4);
			if(tmp != read_config)
			{
				//handle error
				printf("[DS2482_write_config()] - tmp = %d \n", tmp);
				DS2482_reset();
				return 0;
			}
			else if(tmp == read_config)
			{
				return 1;
			}
		case ESP_ERR_INVALID_ARG:
			printf("[DS2482_write_config()] - Parameter error (2) \n");
			return 0;
		case ESP_FAIL:
			printf("[DS2482_write_config()] - Sending command error, slave doesn`t ACK the transfer \n");
			return 0;
		case ESP_ERR_INVALID_STATE:
			printf("[DS2482_write_config()] - i2c driver not installed or not in master mode \n");
			return 0;
		case ESP_ERR_TIMEOUT:
			printf("[DS2482_write_config()] - Operation timeout because the bus is busy \n");
			return 0;
		default:
			printf("[DS2482_write_config()] - default block");
			return 0;
	}
}




uint8_t DS2482_detect()
{
	if(!DS2482_reset())
	{
		printf("[DS2482_detect()] - ds2482 not detected or failure to perform reset \n");
		return 0;
	}
	else
	{
		printf("[DS2482_detect()] - ds2482 was reset \n");
	}
	
	// default configuration
	c1WS = 0;
	cSPU = 0;
	cPPM = 0;
	cAPU = 1;
	
	if(!DS2482_write_config(c1WS | cSPU | cPPM |cAPU))
	{
		printf("[DS2482_detect()] - ds2482 failure to write configuration byte \n");
		return 0;
	}
	else
	{
		printf("[DS2482_detect()] - ds2482 was written \n");
	}
	return 1;
}





uint8_t OWReset(void)
{
	/*
	 * S AD,0 [A] 1WRS [A] P S AD,1 [A] [Status] A [Status] A\ P
	 *									\--------/
	 *						Repeat until 1WB bit has changed to 0
	*/
	uint8_t status = 0;
	uint8_t *st;
	uint8_t SD;
	uint8_t PPD;
	int poll_count = 0;
	
	st = &status;
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);  //S
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0  - [A]);
	i2c_master_write_byte(cmd, CMD_1WRS, ACK_CHECK_EN); //1WRS - [A]
	i2c_master_stop(cmd); // P
	esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	switch(ret){
		case ESP_OK:
			printf("[OWReset()] - CMD_1WRS = OK \n");
			break;
		case ESP_ERR_INVALID_ARG:
			printf("[OWReset()] - Parameter error (1) \n");
			return 0;
		case ESP_FAIL:
			printf("[OWReset()] - Sending command error, slave doesn`t ACK the transfer \n");
			return 0;
		case ESP_ERR_INVALID_STATE:
			printf("[OWReset()] - i2c driver not installed or not in master mode \n");
			return 0;
		case ESP_ERR_TIMEOUT:
			printf("[OWReset()] - Operation timeout because the bus is busy \n");
			return 0;
		default:
			printf("[OWReset()] - default block");
			return 0;
	}
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd); //S
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1  - [A]
	do
	{
		i2c_master_read_byte(cmd, st, ACK_VAL); //[Status] A
	}
	while ((*st & STATUS_1WB) && (poll_count++ < POLL_LIMIT)); //Repeat untill 1WB bit has changed to 0
	i2c_master_read_byte(cmd, st, NACK_VAL); //[Status] notA
	i2c_master_read_byte(cmd, &read_config, NACK_VAL); // [CF] - notA
	i2c_master_stop(cmd); // P
	ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	switch(ret){
		case ESP_OK:
			if(*st & STATUS_PPD)
			{
				printf("[OWReset()] - PPD = %d \n", PPD);
				PPD = 1;
			}
			else
			{
				PPD = 0;
				printf("[OWReset()] - PPD = %d \n", PPD);
			}
			
			if(*st & STATUS_SD)
			{
				printf("[OWReset()] - SD = %d \n", SD);
				SD = 1;
			}
			else
			{
				SD = 0;
				printf("[OWReset()] - SD = %d \n", SD);
			}
		case ESP_ERR_INVALID_ARG:
			printf("[OWReset()] - Parameter error (2) \n");
			return 0;
		case ESP_FAIL:
			printf("[OWReset()] - Sending command error, slave doesn`t ACK the transfer \n");
			return 0;
		case ESP_ERR_INVALID_STATE:
			printf("[OWReset()] - i2c driver not installed or not in master mode \n");
			return 0;
		case ESP_ERR_TIMEOUT:
			printf("[OWReset()] - Operation timeout because the bus is busy \n");
			return 0;
		default:
			printf("[OWReset()] - default block");
			return 0;
	}
	
	if(poll_count >=POLL_LIMIT)
	{
		DS2482_reset();
		return 0;
	}
	
	if(PPD)
		return 1;
	else
		return 0;
}




uint8_t OWTouchBit(uint8_t sendbit)
{
	/*
	 * S AD,0 [A] 1WSB [A] BB [A] P S AD,1 [A] [Status] A [Status] A\ P
	 *											\--------/
	 *							Repeat until 1WB bit has changed to 0
	*/
	uint8_t status = 0;
	uint8_t *st;
	int poll_count = 0;
	
	st = &status;
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);  //S
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); //AD,0  - [A]);
	i2c_master_write_byte(cmd, CMD_1WSB, ACK_CHECK_EN); //1WSB - [A]
	i2c_master_write_byte(cmd, sendbit ? 0x80 : 0x00, ACK_CHECK_EN); //BB - [A]
	i2c_master_stop(cmd); // P
	esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	switch(ret){
		case ESP_OK:
			printf("[OWTouchBit()] - CMD_1WSB = OK \n");
			break;
		case ESP_ERR_INVALID_ARG:
			printf("[OWTouchBit()] - Parameter error (1) \n");
			return 0;
		case ESP_FAIL:
			printf("[OWTouchBit()] - Sending command error, slave doesn`t ACK the transfer \n");
			return 0;
		case ESP_ERR_INVALID_STATE:
			printf("[OWTouchBit()] - i2c driver not installed or not in master mode \n");
			return 0;
		case ESP_ERR_TIMEOUT:
			printf("[OWTouchBit()] - Operation timeout because the bus is busy \n");
			return 0;
		default:
			printf("[OWTouchBit()] - default block");
			return 0;
	}
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd); //S
	i2c_master_write_byte(cmd, DS2482_ADDR << 1 | READ_BIT, ACK_CHECK_EN); //AD,1  - [A]
	do
	{
		i2c_master_read_byte(cmd, st, ACK_VAL); //[Status] A
	}
	while ((*st & STATUS_1WB) && (poll_count++ < POLL_LIMIT)); //Repeat untill 1WB bit has changed to 0
	i2c_master_read_byte(cmd, st, NACK_VAL); //[Status] notA
	i2c_master_read_byte(cmd, &read_config, NACK_VAL); // [CF] - notA
	i2c_master_stop(cmd); // P
	ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	switch(ret){
		case ESP_OK:
			printf("[OWTouchBit()] - *st = %d \n", *st);
			break;
		case ESP_ERR_INVALID_ARG:
			printf("[OWTouchBit()] - Parameter error (2) \n");
			return 0;
		case ESP_FAIL:
			printf("[OWTouchBit()] - Sending command error, slave doesn`t ACK the transfer \n");
			return 0;
		case ESP_ERR_INVALID_STATE:
			printf("[OWTouchBit()] - i2c driver not installed or not in master mode \n");
			return 0;
		case ESP_ERR_TIMEOUT:
			printf("[OWTouchBit()] - Operation timeout because the bus is busy \n");
			return 0;
		default:
			printf("[OWTouchBit()] - default block");
			return 0;
	}
	
	if(poll_count >=POLL_LIMIT)
	{
		DS2482_reset();
		return 0;
	}
	
	if(*st & STATUS_SBR)
		return 1;
	else
		return 0;
}





























