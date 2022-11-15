// ----------------------------------------------------------------------------
// main.c
// ----------------------------------------------------------------------------

//***** Header Files **********************************************************
// XDC module Header Files
#include <xdc/std.h>                                                            // XDC "base types" - must be included FIRST
#include <xdc/runtime/Types.h>                                                  // XDC constants/types used in xdc.runtime pkg
#include <xdc/cfg/global.h>                                                     // For all BIOS instances created statically in RTOS .cfg file
#include <xdc/runtime/Error.h>                                                  // For error handling (e.g. Error block)
#include <xdc/runtime/System.h>                                                 // XDC System functions (e.g. System_abort(), System_printf())

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>               // BIOS kernel APIs
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
// Standard C Header Files
#include <stdint.h>                                                             // Standard integer types
#include <stddef.h>                                                             // Standard variable types and macros

// Peripheral Driver Header Files
#include "ti_drivers_config.h"                                                  // Syscfg Board/Drivers Header file
#include "myAdc.h"
#include "myUart.h"
#include "myGpio.h"
#include "mpu6050.h"
#include "myMailbox.h"
#include <ti/drivers/UART.h>
extern  UART_Handle uart0;

Error_Block         Eb;

Mailbox_Params      mbxParams;
Mailbox_Handle      mbHandle_semg, mbHandle_mpu6050, mbHandle_printData;

Clock_Handle        clk1Handle;
Clock_Params        clkParams;

//*****************************************************************************
// Main
//*****************************************************************************
void main (void)
{
    // Initialize Peripherals
    Board_init();                                                               // Call TI Driver config init function
    myGpio_init();
    myUart_init();
    myADC_init();
    MPU6050_I2C_init();

    // Init Error Block
    Error_init(&Eb);

    // Create Mailbox
    Mailbox_Params_init(&mbxParams);
    mbHandle_mpu6050 = Mailbox_create( sizeof(MsgMPU6050Data) ,                              // bufsize (size of INT)
                                    2,                                              // number of buffers = 2 (just the LED state, 2nd one is unused)
                                    &mbxParams,                                     // params
                                    &Eb);                                  // error block passed and pointer checked vs. NULL below
    if (Error_check(&Eb))
    {                                                      // If NULL handle, abort
           System_abort("mailbox create failed");
    }

    mbHandle_semg = Mailbox_create( sizeof(MsgSemgData) ,                              // bufsize (size of INT)
                                       2,                                              // number of buffers = 2 (just the LED state, 2nd one is unused)
                                       &mbxParams,                                     // params
                                       &Eb);                                  // error block passed and pointer checked vs. NULL below
   if (Error_check(&Eb))
   {                                                      // If NULL handle, abort
          System_abort("mailbox create failed");
    }


   mbHandle_printData = Mailbox_create( sizeof(MsgPrintData) ,                              // bufsize (size of INT)
                                         10,                                              // number of buffers = 2 (just the LED state, 2nd one is unused)
                                         &mbxParams,                                     // params
                                         &Eb);                                  // error block passed and pointer checked vs. NULL below
     if (Error_check(&Eb))
     {                                                      // If NULL handle, abort
            System_abort("mailbox create failed");
      }

    // Start TI-RTOS kernel (which includes the background loop)

    BIOS_start();                                                               // Start TI-RTOS (SYS/BIOS kernel) services - it does not return
}

/* ===========================================================================
   Copyright (c) 2022 Embedded Advantage LLC (www.embeddedadvantage.com)

   This software can be re-used, for personal or commercial purposes,
   provided that the conditions described below are met:

   a. Redistribution of unmodified source code outside of the license
      owner's (individual who paid for training from Embedded Advantage)
      company is prohibited. Source code may be used to create derivative
      works.
   b. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
============================================================================ */
