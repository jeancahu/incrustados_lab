//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include "msp.h"
#include "main.hpp"
#include "HAL_OPT3001.h"
// ****************
// Global variables
// ****************
uint32_t g_counter;
uint32_t g_reboundCtl_counter;
bool g_ctrButton;

// Lamp status
bool g_lamp_status;

void main(void)
{
  // - Run the overall setup function for the system
  setup();
  systemConfirm();
  while (1) {
    // LAMP = 0x0000;
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

  // TODO CONFIG P1
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
  // - Configure ADC1
  // - Select the ADC input channel and the ADC clock division
  //P5->SEL1 |= BIT4;                           // Configure P5.4 for ADC
  //P5->SEL0 |= BIT4;


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
    P2->OUT ^= BIT0 | BIT1 | BIT2; // - Toggle
  }
  return;
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
    else if (( P3->IN & BIT5 ) && ( g_ctrButton ))
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
      }
    // Clear interrupt flag for all terminals
    P3->IFG &= ~(BIT5);
    // Enable P3.6 interrupt
    P3->IE |= (BIT5);  
    return;
  }

  // ADC14 interrupt service routine
  //void ADC14IsrHandler(void) {}
}
