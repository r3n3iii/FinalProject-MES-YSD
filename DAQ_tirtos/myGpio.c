// ----------------------------------------------------------------------------
// myGpio.c
// ----------------------------------------------------------------------------

//***** Header Files **********************************************************
// Standard C Header Files
#include <stdint.h>                                                             // Standard integer types
#include <stddef.h>                                                             // Standard variable types and macros
#include <stdio.h>                                                              // Standard I/O for printf()

// Peripheral Driver Header Files
#include "ti_drivers_config.h"                                                  // Syscfg Board/Drivers Header file
#include <ti/drivers/GPIO.h>                                                    // TI GPIO Driver header file
#include "myGpio.h"                                                             // Header file for myGpio.c


//***** Prototypes ************************************************************
void gpioButtonFxn0(uint_least8_t);
void gpioButtonFxn1(uint_least8_t);

//*****************************************************************************
// Initialize GPIO
//
// The callback function, specified by GPIO_setCallback() function, lets you
// specify how your system should respond to the associated GPIO interrupt.
// Internally the GPIO Driver contains an interrupt service routine (ISR) which
// handles the interrupt's administrative details (e.g. clears flag bits,
// re-enables interrupt, plugs vector table) as well as calling the specified
// "callback" function.
//
// Notes:
//   - CONFIG_GPIO_LED_0 is connected to "LED1" on the LaunchPad
//   - CONFIG_GPIO_LED_1 is connected to "LED2_RED" on the LaunchPad
//   - CONFIG_GPIO_LED_2 is connected to "LED2_GREEN" on the LaunchPad
//   - CONFIG_GPIO_LED_3 is connected to "LED2_BLUE" on the LaunchPad
//   - CONFIG_GPIO_BUTTON_0 is connected to Switch 1 (SW1) on the LaunchPad
//   - CONFIG_GPIO_BUTTON_1 is connected to Switch 2 (SW2) on the LaunchPad
//*****************************************************************************
void myGpio_init(void)
{
    // Call driver init function
    GPIO_init();                                                                // Call GPIO driver initialization function

    // Configure pins
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);     // Configure "LED_0" as an output and set it low (i.e. '0')
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);     // Configure "LED_1" as an output and set it low (i.e. '0')
    GPIO_setConfig(CONFIG_GPIO_LED_2, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);     // Configure "LED_2" as an output and set it low (i.e. '0')
    GPIO_setConfig(CONFIG_GPIO_LED_3, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);     // Configure "LED_3" as an output and set it low (i.e. '0')


    GPIO_setConfig(CONFIG_GPIO_BUTTON_1,                                        // Configure I/O pin for Button1
                   GPIO_CFG_IN_PU                                               // Configure as input pin with internal pull-up
                   | GPIO_CFG_IN_INT_FALLING                                    // Configure interrupt to trigger on falling edge
    );
    // Turn on user LEDs
   // GPIO_write(CONFIG_GPIO_LED_0, CONFIG_LED_ON);                               // Turn on "LED_0" (i.e. set its pin to '1')


    // Install button interrupt callback functions

    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);                     // When "SW2" is pressed, call 'gpioButtonFxn1'

    // Enable interrupts

    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);  // Enable individual interrupt for Board_GPIO_BUTTON1

    printf("GPIO has been initialized\n");
}
//*****************************************************************************
// gpioButtonFxn0
//
// Callback function for the GPIO interrupt on Board_GPIO_BUTTON0
//
// Notes:
//   - CONFIG_GPIO_BUTTON_0 is connected to Switch 1 (SW1) on the LaunchPad
//   - The "index" argument is not used by this function, but it references
//     which GPIO input was triggered as defined by the GPIOName provided
//     in the project's board specific header file
//*****************************************************************************
void gpioButtonFxn0(uint_least8_t index)
{
    GPIO_toggle(CONFIG_GPIO_LED_0);                                             // Toggle LED
}

//*****************************************************************************
// gpioButtonFxn1
//
// Callback function for the GPIO interrupt on Board_GPIO_BUTTON1
//
// Notes:
//   - CONFIG_GPIO_BUTTON_1 is connected to Switch 2 (SW2) on the LaunchPad
//   - The "index" argument is not used by this function, but it references
//     which GPIO input was triggered as defined by the GPIOName provided
//     in the project's board specific header file
//*****************************************************************************
void gpioButtonFxn1(uint_least8_t index)
{
    GPIO_toggle(CONFIG_GPIO_LED_1);                                             // Toggle LED
}
