// ----------------------------------------------------------------------------
// myIdleFxn.c
//
// This Idle function is called by the Idle thread via the .cfg file in the
// referenced project. It is simply a placeholder so that our solution files
// can use a single .cfg file for all solution projects whether they use an
// Idle function or not.
// ----------------------------------------------------------------------------
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

//*****************************************************************************
// Idle Fxn - called by Idle via .cfg Idle module (statically added to .cfg)
//*****************************************************************************
void myIdleFxn (void)
{
    MAP_PCM_gotoLPM0();
    // dummy function
}

