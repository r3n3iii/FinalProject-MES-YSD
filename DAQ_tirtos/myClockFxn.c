// ----------------------------------------------------------------------------
// myClockFxn.c
// ----------------------------------------------------------------------------

//***** Header Files **********************************************************
// XDC module Header Files
#include <xdc/std.h>                                                            // XDC "base types" - must be included FIRST
#include <xdc/runtime/Types.h>                                                  // XDC constants/types used in xdc.runtime pkg
#include <xdc/cfg/global.h>                                                     // For all BIOS instances created statically in RTOS .cfg file
#include <xdc/runtime/Log.h>                                                    // For any Log_info() call

// TI-RTOS Kernel Header Files
#include <ti/sysbios/BIOS.h>                                                    // BIOS module API
#include <ti/sysbios/knl/Task.h>                                                // Task APIs
#include <ti/sysbios/knl/Semaphore.h>                                           // Semaphore APIs
#include <ti/sysbios/knl/Clock.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
// Standard C Header Files
#include <stdint.h>                                                             // Standard integer types
#include <stddef.h>                                                             // Standard variable types and macros
#include <ti/drivers/GPIO.h>

//***** Prototypes ************************************************************
void HeartBeatFxn(void);                                                                 // Callback used for toggling the LED
void SlowClockFnx(void);

//*****************************************************************************
// Clock Function
//
// This function is called when 500 system "ticks" expire assuming that the
// Clock Module (in the .cfg) is set to 1000uS per tick (which is the
// default). This Clock Function simply posts a Semaphore to unblock the
// Task to toggle the LED.
//
//*****************************************************************************
void HeartBeatFxn()
{
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
    Semaphore_post(SEM_semg);
}

void SlowClockFnx()
{
    Semaphore_post(SEM_mpu6050);
    Semaphore_post(SEM_cli);
}
