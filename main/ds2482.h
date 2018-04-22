/*
 * ds2482.h
 *
 *  Created on: 2018/04/21
 *      Author: zip1982b
 */
 
#ifndef MAIN_DS2482_H_
#define MAIN_DS2482_H_

#include "driver/i2c.h"


#define DS2482_ADDR			0x18		/* slave address for DS2482 bridge*/

#define WRITE_BIT			I2C_MASTER_WRITE /* I2C Master write */
#define READ_BIT			I2C_MASTER_READ  /* I2C_MASTER_READ */
#define ACK_CHECK_EN		0x1				 /* I2C master will check ack from slave */
#define ACK_CHECK_DIS		0x0				 /* I2C master will not check ack from slave */
#define ACK_VAL				0x0				 /* I2C ack value */
#define NACK_VAL			0x1				 /* I2C nack value */

#define POLL_LIMIT			24

/* Masks STATUS register ds2482 */
/* |DIR|TSB|SBR|RST|LL|SD|PPD|1WB| */
#define STATUS_DIR			0x80			/*10000000b - Branch Direction taken*/
#define STATUS_TSB			0x40			/*01000000b - Triplet second bit*/
#define STATUS_SBR			0x20			/*00100000b - Single bit result*/
#define STATUS_RST			0x10			/*00010000b - Device reset*/
#define STATUS_LL			0x8				/*00001000b - Logic level*/
#define STATUS_SD			0x4				/*00000100b - Short detected*/
#define STATUS_PPD			0x2				/*00000010b - Presence pulse detected*/
#define STATUS_1WB			0x1				/*00000001b - 1-wire busy*/

#define CONFIG_APU			0x1

#define CMD_DRST			0xF0			//command "device ds2482 reset"
#define CMD_WCFG			0xD2			//command "Write configuration"

#define CMD_1WRS			0xB4			//command "1-wire reset"
#define CMD_1WSB			0x87			//command "1-wire single bit"
#define CMD_1WWB			0xA5			//command "1-wire write byte"
#define CMD_1WRB			0x96			//command "1-wire read byte"
#define CMD_1WT				0x78			//command "1-wire triplet"
#define CMD_SRP				0xE1			//command "Set read pointer"


#define SearchROM			0xF0			//the search command that all 1-wire devices respond.


uint8_t crc_table[] = {
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
}




/**
* @brief Perform a device reset on the DS2482
* 
* @return 
*	 1: if device (ds2482) was reset
*	 0: device (ds2482) not detected or failure to perform reset
*/
uint8_t DS2482_reset(void);






/**
* @brief Write the configuration register in the DS2482.
* @param uint8_t config -> DS2482_write_config(c1WS | cSPU | cPPM | cAPU)
* @return
*		 1: config written and response correct 
*		 0: resonse incorrect
*/
uint8_t DS2482_write_config(uint8_t config);





/**
* @brief Detecting ds2482 and writing to the configuration registry the default value.
* @param void
* @return
* 		1: if device (ds2482) was detected and written
* 		0: device not detected or failure to write configuration byte
*/
uint8_t DS2482_detect(void);





/**
* @brief Reset all of the devices on the 1-wire net and return the result.
* @param void
* @return
* 		1: presence pulse(s) detected, device(s) reset
* 		0: no presence pulses detected
*/
uint8_t OWReset(void);




/**
* @brief Send bit of communication to the 1-wire net and return the result 1 bit read
* 			from the 1-wire net.
* @param uint8_t sendbit 
* @return 
* 		0: 0 bit read from sendbit
* 		1: 1 bit read from sendbit
*/
uint8_t OWTouchBit(uint8_t sendbit);





/**
* @brief Send 1 bit of communication to the 1-wire net.
* @param uint8_t sendbit - least significant bit is used.
* @return OWTouchBit(sendbit)
*/
uint8_t OWWriteBit(uint8_t sendbit);




/**
* @brief Send 1 bit of communication to the 1-wire net.
* @param void.
* @return OWTouchBit(0x01)
*/
uint8_t OWReadBit(void);






/**
* @brief Send 8 bits of communication to the 1-wire net and verify that the 
* 			8 bits read the 1-wire net are the same (write operation).
* @param uint8_t sendbyte - 8 bits to send.
* @return 
* 		1: bytes written and echo was the same
* 		0: echo was not the same
*/
void OWWriteByte(uint8_t sendbyte);





/**
* @brief Send 8 bits of read communication to the 1-wire net and return the 
* 			result 8 bits read from the 1-wire net.
* @param void.
* @return 8 bits read from 1-wire net.
*/
uint8_t OWReadByte(void);




/**
* @brief Send 8 bits of communication to the 1-wire net and return the 
* 			result 8 bits read from the 1-wire net.
* @param uint8_t sendbyte - 8 bits send.
* @return 8 bits read from sendbyte.
*/
uint8_t OWTouchByte(uint8_t sendbyte);





/**
* @brief Transfers a block of data to and from the 1-wire net.
* @param uint8_t *tran_buf - pointer to a block of uint8_t of length 'tran_len'
* 			that will be sent to the 1-wire net.
* @param int tran_len - length in bytes to transfer.
* @return void.
*/
void OWBlock(uint8_t *tran_buf, int tran_len);



/**
* @brief Use the ds2482 help command "1-wire triplet" to perform one bit of a 1-wire search.
* @param uint8_t search_direction - in case of discrepancy the search_direction.
* @return The ds2482 status byte result from the triplet command.
*/
uint8_t DS2482_search_triplet(uint8_t search_direction);




/**
* @brief Calculate the CRC8 of the byte value provided with the current global 'crc8' value.
* @param uint8_t value. 
* @return current global crc8 value.
*/
uint8_t calc_crc8(uint8_t value);




/**
* @brief general search.
* @param void. 
* @return
* 			1: When a 1-wire device was found and its serial number placed in the global ROM.
* 			0: When no new device was found.
*/
uint8_t OWSearch(void);



/**
* @brief Find the 'first' devices on the 1-wire net.
* @param void. 
* @return
* 			1: device found, ROM number in ROM_NO buffer.
* 			0: no device present.
*/
uint8_t OWFirst(void);




/**
* @brief Find the 'next' devices on the 1-wire net.
* @param void. 
* @return
* 			1: device found, ROM number in ROM_NO buffer.
* 			0: device not found, end of search.
*/
uint8_t OWNext(void);





#endif