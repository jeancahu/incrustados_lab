//*****************************************************************************
//
// MSP432 prototype implementation of night lamp for kids
//
//****************************************************************************


#include "main.hpp"
#include "msp.h"

#include "HAL_OPT3001.h"

#include "i2c.h"
#include "adc14.h"
#include "gpio.h"
// ****************
// Global variables
// ****************
uint32_t g_counter;
uint32_t g_reboundCtl_counter;
uint32_t g_counterMicSampler;
uint8_t  g_noiseBuffer;


bool g_ctrButton;
bool g_light_flag;
bool g_noise_flag;


void main(void)
{
  // - Run the overall setup function for the system
  setup();
  systemConfirm();
  startADC();
  while (1) {
    if ( g_noiseBuffer > 0x0F )
      {
    	g_noise_flag = true;
      } else {
      g_noise_flag = false;

    }
    if ( OPT3001_getLux() > LUX_LIMIT )
      {
	g_light_flag = true;
      } else {
      g_light_flag = false;
    }
  };
}

// **********************************
// Setup function for the application
// @input - none
// @output - none
// **********************************
static void setup(void)
{
  // ****************************
  //         DEVICE CONFIG
  // ****************************
  // - Disable WDT
  WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

  // ****************************
  //     Define Global Vars
  // ****************************
  g_counter = ZERO;
  g_reboundCtl_counter = ZERO;
  g_ctrButton = false;

  // ****************************
  //         PORT CONFIG
  // ****************************
  // - P2.{0,1,2} is connected to the RGB LED
  P2->DIR = BIT0|BIT1|BIT2; //RGB LED
  P2->OUT = ZERO;

  P3->OUT = ZERO;
  P3->DIR &= ~(BIT5); // Set bit0 as input
  P3->REN |= BIT5; // Enable internal pull-up/down resistors
  P3->OUT |= BIT5; // Select pull-up mode
  P3->IES &= ~(BIT5); // hi/lo edge
  P3->IFG &= ~(BIT5);
  P3->IE |= BIT5; // Enable P3.5 interrupt

  NVIC_SetPriority(PORT3_IRQn,1);
  NVIC_EnableIRQ(PORT3_IRQn);
  // ****************************
  //       TIMER CONFIG
  // ****************************
  // - Configure Timer32_1  with MCLK (3Mhz),
  //   Division by 1, Enable the interrupt, Periodic Mode
  // - Enable the interrupt in the NVIC
  // - Start the timer in UP mode.
  // - Re-enable interrupts
  TIMER32_1->LOAD = TIMER32_COUNT; //~1ms ---> a 3Mhz
  TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE |\
    TIMER32_CONTROL_PRESCALE_0 |\
    TIMER32_CONTROL_MODE |\
    TIMER32_CONTROL_IE |\
    TIMER32_CONTROL_ENABLE;
  NVIC_SetPriority(T32_INT1_IRQn,2);
  NVIC_EnableIRQ(T32_INT1_IRQn);

  // ****************************
  //       ADC CONFIG
  // ****************************
  initADC();
  initMicrophone();

  g_noiseBuffer = ZERO;


  // ****************************
  //         I2C CONFIG
  // - Initialize I2C communication */
  // ****************************
  Init_I2C_GPIO();
  I2C_init();
  // ****************************
  //         DEVICE CONFIG
  // - Initialize OPT3001 digital ambient light sensor */
  // ****************************
  OPT3001_init();


  LAMP = LAMP_1 | LAMP_2 | LAMP_3;

  // Enables all interrupts
  __enable_irq();

  return;
}

/* Required system-up confirmation to user by blinking LED 3 times */
static void systemConfirm ()
{
  for ( int i = 0 ; i<6 ; i++ ){
    g_counter = 0x00000300;
    while ( g_counter ) {};
    LAMP ^= BIT0 | BIT1 | BIT2; // - Toggle
  }

  if (g_light_flag)
    {
	  LAMP = ZERO;
    }
  else
    {
	  LAMP = LAMP_1 | LAMP_2 | LAMP_3;
    }
  return;
}


static void initADC() {
  ADC14_enableModule();

  // This sets the conversion clock to 3MHz
  ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC,
		   ADC_PREDIVIDER_1,
		   ADC_DIVIDER_1,
		   0
		   );

  ADC14_setResolution(ADC_14BIT);

  // This configures the ADC to store output results
  // in ADC_MEM0 (single-channel conversion, repeat mode)
  ADC14_configureSingleSampleMode(ADC_MEM0, true);

  // This configures the ADC in automatic conversion mode
  ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
}

void startADC() {
  // The ADC is in continuous sampling mode, so after calling this once
  // the ADC will continuously update
  ADC14_enableConversion();
  ADC14_toggleConversionTrigger();
}

void initMicrophone() {
  // This configures ADC_MEM0 to store the result from
  // input channel A10 (Microphone), in non-differential input mode
  // (non-differential means: only a single input pin)
  // The reference for Vref- and Vref+ are VSS and VCC respectively
  ADC14_configureConversionMemory(ADC_MEM0,
				  ADC_VREFPOS_AVCC_VREFNEG_VSS,
				  ADC_INPUT_A10,
				  ADC_NONDIFFERENTIAL_INPUTS);

  // This selects the GPIO as analog input
  // A9 is multiplexed on GPIO port P4 pin PIN4
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
					     GPIO_PIN3,
					     GPIO_TERTIARY_MODULE_FUNCTION);
}

unsigned getSampleMicrophone() {
  return ADC14_getResult(ADC_MEM0);
}



/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig =
  {
    EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
    48000000,                               // SMCLK = 48MHz
    EUSCI_B_I2C_SET_DATA_RATE_400KBPS,      // Desired I2C Clock of 100khz
    0,                                      // No byte counter threshold
    EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
  };

void Init_I2C_GPIO()
{
  /* Select I2C function for I2C_SCL(P6.5) & I2C_SDA(P6.4) */
  GPIO_setAsPeripheralModuleFunctionOutputPin(
					      GPIO_PORT_P6,
					      GPIO_PIN5,
					      GPIO_PRIMARY_MODULE_FUNCTION);

  GPIO_setAsPeripheralModuleFunctionOutputPin(
					      GPIO_PORT_P6,
					      GPIO_PIN4,
					      GPIO_PRIMARY_MODULE_FUNCTION);
}


/***************************************************************************
 * @brief  Configures I2C
 * @param  none
 * @return none
 ******************************************************************************/

void I2C_init(void)
{
  /* Initialize USCI_B0 and I2C Master to communicate with slave devices*/
  I2C_initMaster(EUSCI_B1_BASE, &i2cConfig);

  /* Disable I2C module to make changes */
  I2C_disableModule(EUSCI_B1_BASE);

  /* Enable I2C Module to start operations */
  I2C_enableModule(EUSCI_B1_BASE);

  return;
}


/***************************************************************************
 * @brief  Reads data from the sensor
 * @param  writeByte Address of register to read from
 * @return Register contents
 ******************************************************************************/

int I2C_read16(unsigned char writeByte)
{
  volatile int val = 0;
  volatile int valScratch = 0;

  /* Set master to transmit mode PL */
  I2C_setMode(EUSCI_B1_BASE,
	      EUSCI_B_I2C_TRANSMIT_MODE);

  /* Clear any existing interrupt flag PL */
  I2C_clearInterruptFlag(EUSCI_B1_BASE,
			 EUSCI_B_I2C_TRANSMIT_INTERRUPT0);

  /* Wait until ready to write PL */
  while (I2C_isBusBusy(EUSCI_B1_BASE));

  /* Initiate start and send first character */
  I2C_masterSendMultiByteStart(EUSCI_B1_BASE, writeByte);

  /* Wait for TX to finish */
  while(!(I2C_getInterruptStatus(EUSCI_B1_BASE,
				 EUSCI_B_I2C_TRANSMIT_INTERRUPT0)));

  /* Initiate stop only */
  I2C_masterSendMultiByteStop(EUSCI_B1_BASE);

  /* Wait for Stop to finish */
  while(!I2C_getInterruptStatus(EUSCI_B1_BASE,
				EUSCI_B_I2C_STOP_INTERRUPT));

  /*
   * Generate Start condition and set it to receive mode.
   * This sends out the slave address and continues to read
   * until you issue a STOP
   */
  I2C_masterReceiveStart(EUSCI_B1_BASE);

  /* Wait for RX buffer to fill */
  while(!(I2C_getInterruptStatus(EUSCI_B1_BASE,
				 EUSCI_B_I2C_RECEIVE_INTERRUPT0)));

  /* Read from I2C RX register */
  val = I2C_masterReceiveMultiByteNext(EUSCI_B1_BASE);

  /* Receive second byte then send STOP condition */
  valScratch = I2C_masterReceiveMultiByteFinish(EUSCI_B1_BASE);

  /* Shift val to top MSB */
  val = (val << 8);

  /* Read from I2C RX Register and write to LSB of val */
  val |= valScratch;

  /* Return temperature value */
  return (int16_t)val;
}


/***************************************************************************
 * @brief  Writes data to the sensor
 * @param  pointer  Address of register you want to modify
 * @param  writeByte Data to be written to the specified register
 * @return none
 ******************************************************************************/

void I2C_write16 (unsigned char pointer, unsigned int writeByte)
{
  /* Set master to transmit mode PL */
  I2C_setMode(EUSCI_B1_BASE,
	      EUSCI_B_I2C_TRANSMIT_MODE);

  /* Clear any existing interrupt flag PL */
  I2C_clearInterruptFlag(EUSCI_B1_BASE,
			 EUSCI_B_I2C_TRANSMIT_INTERRUPT0);

  /* Wait until ready to write PL */
  while (I2C_isBusBusy(EUSCI_B1_BASE));

  /* Initiate start and send first character */
  I2C_masterSendMultiByteStart(EUSCI_B1_BASE,
			       pointer);

  /* Send the MSB to SENSOR */
  I2C_masterSendMultiByteNext(EUSCI_B1_BASE,
			      (unsigned char)(writeByte>>8));

  I2C_masterSendMultiByteFinish(EUSCI_B1_BASE,
				(unsigned char)(writeByte&0xFF));

}


void I2C_setslave(unsigned int slaveAdr)
{
  /* Specify slave address for I2C */
  I2C_setSlaveAddress(EUSCI_B1_BASE,
		      slaveAdr);

  /* Enable and clear the interrupt flag */
  I2C_clearInterruptFlag(EUSCI_B1_BASE,
			 EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0);
  return;
}


void OPT3001_init()
{
  /* Specify slave address for OPT3001 */
  I2C_setslave(OPT3001_SLAVE_ADDRESS);
  /* Set Default configuration for OPT3001*/
  I2C_write16(CONFIG_REG, DEFAULT_CONFIG_100);
}

unsigned long int OPT3001_getLux()
{
  /* Specify slave address for OPT3001 */
  I2C_setslave(OPT3001_SLAVE_ADDRESS);

  uint16_t exponent = 0;
  uint32_t result = 0;
  int16_t raw;
  raw = I2C_read16(RESULT_REG);
  /*Convert to LUX*/
  //extract result & exponent data from raw readings
  result = raw&0x0FFF;
  exponent = (raw>>12)&0x000F;
  //convert raw readings to LUX
  switch(exponent){
  case 0: //*0.015625
    result = result>>6;
    break;
  case 1: //*0.03125
    result = result>>5;
    break;
  case 2: //*0.0625
    result = result>>4;
    break;
  case 3: //*0.125
    result = result>>3;
    break;
  case 4: //*0.25
    result = result>>2;
    break;
  case 5: //*0.5
    result = result>>1;
    break;
  case 6:
    result = result;
    break;
  case 7: //*2
    result = result<<1;
    break;
  case 8: //*4
    result = result<<2;
    break;
  case 9: //*8
    result = result<<3;
    break;
  case 10: //*16
    result = result<<4;
    break;
  case 11: //*32
    result = result<<5;
    break;
  }
  return result;
}


extern "C"
{
  // - Handle the Timer32 Interrupt
  void T32_INT1_IRQHandler(void)
  {
    TIMER32_1->INTCLR = 0U;

    if ( g_counter )
      { g_counter--; }
    else { LAMP = ZERO; }

    if ( g_reboundCtl_counter )
      { g_reboundCtl_counter--; }
    else if ( (( P3->IN & BIT5 ) && ( g_ctrButton )) |\
	      ( g_noise_flag & !( LAMP & (LAMP_1|LAMP_2|LAMP_3)) &\
		!g_light_flag))
      {
	if ( g_counter )
	  {
	    LAMP = ZERO;
	    g_counter = ZERO;
	  }
	else if ( LAMP & ( LAMP_1 | LAMP_2 | LAMP_3 ) )
	  {
	    LAMP = ZERO;
	  } else {
	  // Turn on lamp
	  LAMP = LAMP_1 | LAMP_2 | LAMP_3;
	  g_counter = HALF_HOUR;
	}
	g_ctrButton = false;
      }

    // Microphone
    if ( g_counterMicSampler )
      {
    	g_counterMicSampler--;
      }
    else
      {
    	g_counterMicSampler = SOUND_TIMER_INT;


    	if ( getSampleMicrophone() > SOUND_LIMIT )
	  {
	    // Silence
            g_noiseBuffer = g_noiseBuffer<<1;
	  } else {
	  // Noise
	  g_noiseBuffer = ( g_noiseBuffer<<1 ) | 0x01;
    	}
      }
    return;
  }

  // - Pulse button ISR
  void PORT3_IRQHandler(void)
  {
    // Disable interrupt for P1.1
    P3->IE &= ~(BIT5);
    if ( !g_reboundCtl_counter )
      {
	// Suppress bounding
	g_reboundCtl_counter = REB_CTL;
	g_ctrButton = true;
	g_noiseBuffer = ZERO;
	g_noise_flag = false;
      }
    // Clear interrupt flag for all terminals
    P3->IFG &= ~(BIT5);
    // Enable P3.6 interrupt
    P3->IE |= (BIT5);
    return;
  }

}
