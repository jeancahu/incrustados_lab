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

//#define HALF_HOUR 1800000 // half-hour in ms
#define HALF_HOUR 5000 // FIX-ME
#define REB_CTL 40 // 40 ms

#define ZERO 0x00000000

// The main Setup function for the application
static void setup(void);
static void systemConfirm(void);

#endif /* MAIN_HPP_ */
