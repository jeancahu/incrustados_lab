 /*
 * main.hpp
 *
 *  Created on: Aug 31, 2016
 *      Author: eortiz
 */

#ifndef MAIN_HPP_
#define MAIN_HPP_
#define TIMER32_COUNT 0x00000BB8  // ~1ms with 3MHz clock

#define LAMP P2->OUT
#define LAMP_1 BIT0
#define LAMP_2 BIT1
#define LAMP_3 BIT2

// 0xFFFF -> Totally Silence;
// 0x0000 -> Totally Noise
//#define SOUND_LIMIT 0x1F99
#define SOUND_LIMIT 0x1F99 // FIX-ME
#define NOISE_ARRAY_LEN 4

#define SOUND_TIMER_INT 625 // Take a sample every 0.625 s
//#define HALF_HOUR 1800000 // half-hour in ms
#define HALF_HOUR 5000 // FIX-ME
#define REB_CTL 40 // 40 ms

#define ZERO 0x00000000

// The main Setup function for the application
static void setup(void);
static void systemConfirm(void);
static void initADC(void);
static void startADC(void);
static void initMicrophone(void);
unsigned getSampleMicrophone(void);

void Init_I2C_GPIO(void);
void I2C_init(void);
int I2C_read16(unsigned char);
void I2C_write16(unsigned char pointer, unsigned int writeByte);
void I2C_setslave(unsigned int slaveAdr);

/*CONSTANTS*/
#define OPT3001_SLAVE_ADDRESS 0x44

#define OPT_INTERRUPT_PIN 11
#define RESULT_REG 0x00
#define CONFIG_REG 0x01
#define LOWLIMIT_REG 0x02
#define HIGHLIMIT_REG 0x03
#define MANUFACTUREID_REG 0x7E
#define DEVICEID_REG 0x7F

#define DEFAULT_CONFIG 0xCC10 // 800ms
#define DEFAULT_CONFIG_100 0xC410 // 100ms

void OPT3001_init(void);
unsigned long int OPT3001_getLux(void);
//unsigned int OPT3001_readManufacturerId(void);
//signed int OPT3001_readDeviceId(void);
//unsigned int OPT3001_readConfigReg(void);
//unsigned int OPT3001_readLowLimitReg(void);
//unsigned int OPT3001_readHighLimitReg(void);

#define LUX_LIMIT 0x00FF

#endif /* MAIN_HPP_ */
