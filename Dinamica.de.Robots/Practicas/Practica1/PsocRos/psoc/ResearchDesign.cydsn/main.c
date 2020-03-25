#define PWM_CW_MAX 400
#define PWM_CW_MIN 315
#define PWM_STOP 300
#define PWM_CCW_MAX 285
#define PWM_CCW_MIN 200
// Tolerance to stop moving motor 
#define TICKS_STOP_QD 500
// For transmitting strings with other variables substituted in,
// (note: re-using variable names since out-of-scope of uart_helper_fcns.)
#define TRANSMIT_LENGTH 128

// Include both the UART helper functions and the header
// that has the global variables we need.
// note that both of these should have include guards in them already
// so it's safe to include them directly here.
#include <project.h>
#include <math.h>
#include <stdlib.h>
#include "stdio.h"
//#include "uart_helper_fcns.h"
#include "data_storage.h"
#include "holaMundo.h"

// for send some debugging messages
char transmit_buffer[TRANSMIT_LENGTH];

// constants of proportionality are integers.

//float Kp_qd = 1;
//float Kp_qd = 0.1;
float Kp_qd = 0.001;
//float Kp_qd = 0;

//float Ki_qd = 1;
//float Ki_qd = 0.00001;
float Ki_qd = 0.00001;

//float Kd_qd = 1;
//float Kd_qd = 0.0005;
float Kd_qd = 0.009;


CY_ISR(timer_handler) { 
    Timer_ReadStatusRegister();
}

int main(void) {
    
    // Enable interrupts for the chip
    CyGlobalIntEnable;
    __enable_irq();
    
    // Start the interrupt handlers / service routines for each interrupt:
    // UART, main control loop, encoder counting.
    // These are found in the corresponding helper files (declarations in .h, implementations in .c)
    isr_UART_StartEx(Interrupt_Handler_UART_Receive);
    isr_Timer_StartEx(timer_handler);
  
    Timer_Start();
    UART_Start();

    
    holaMundo();
    for(;;)
    {
        // Nothing to do. Entirely interrupt driven! Hooray!
    }
}

/* [] END OF FILE */
