/*
 * myADC.c
 *
 *  Created on: Jan 8, 2020
 *      Author: Rene
 */
//********************************************************************************************************

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/driverlib/adc14.h>

//***** Header Files **********************************************************
#include <stdint.h>                                                          // Standard integer types
#include <stddef.h>                                       // Standard variable types and macros
#include <stdio.h>                                        // For printf()
#include <string.h>

// TI-RTOS Kernel Header Files
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

#define REPEAT_MODE true
#define NO_REPEAT false

 //***** Prototypes ************************************************************


 //***** Global Variables ******************************************************
 extern Semaphore_Handle semEMG,semFSR;

 void myADC_init()
{
     /* Enabling the FPU with stacking enabled (for use within ISR) */
     MAP_FPU_enableModule();
     MAP_FPU_enableLazyStacking();

     //![Simple REF Example]
     /* Setting reference voltage to 2.5  and enabling reference */
     MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
     MAP_REF_A_enableReferenceVoltage();

     /* Initializing ADC (MCLK/1/1) */
     MAP_ADC14_enableModule();
     //MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_8, 0);
     MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);

     /* Configuring GPIOs for Analog In */
     MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN7 | GPIO_PIN6| GPIO_PIN5 | GPIO_PIN4 | GPIO_PIN3 | GPIO_PIN2| GPIO_PIN1| GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
     MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5 | GPIO_PIN4| GPIO_PIN3 | GPIO_PIN2 | GPIO_PIN1 | GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
     MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1| GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);

     // Configuring ADC Memory (ADC_MEM0 - ADC_MEM7 (A0 - A7)  with no repeat)
     MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM7, true);
     MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A1, ADC_DIFFERENTIAL_INPUTS); // A0/A1 = P5.4 P5.5
     MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A3, ADC_DIFFERENTIAL_INPUTS); // A2/A3 = P5.3 P5.2
     MAP_ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A5, ADC_DIFFERENTIAL_INPUTS); // A4/A5 = P5.1 P5.0
     MAP_ADC14_configureConversionMemory(ADC_MEM3, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A7, ADC_DIFFERENTIAL_INPUTS); // A6/A7 = P4.7 P4.6
     MAP_ADC14_configureConversionMemory(ADC_MEM4, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A9, ADC_DIFFERENTIAL_INPUTS); // A8/A9 = P4.5 P4.4
     MAP_ADC14_configureConversionMemory(ADC_MEM5, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A11, ADC_DIFFERENTIAL_INPUTS);// A10/A11 = P4.3 P4.2
     MAP_ADC14_configureConversionMemory(ADC_MEM6, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A13, ADC_DIFFERENTIAL_INPUTS);// A12/A13 = P4.1 P4.0
     MAP_ADC14_configureConversionMemory(ADC_MEM7, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A15, ADC_DIFFERENTIAL_INPUTS);// A14/A15 = P6.1 P6.0



     MAP_ADC14_enableInterrupt(ADC_INT1|ADC_INT7);
     MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
     MAP_ADC14_enableConversion();

    // printf("ADC has been initialized\n");
}



