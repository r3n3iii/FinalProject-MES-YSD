// Console IO is a wrapper between the actual in and output and the console code
// In an embedded system, this might interface to a UART driver.

#include "consoleIo.h"
#include <stdio.h>
#include <string.h>
#include "myUart.h"
#include <ti/drivers/UART.h>
#include <stdbool.h>


extern UART_Handle uart0;


eConsoleError ConsoleIoInit(void)
{
    //UART_control(uart1, UART_CMD_ISAVAILABLE,isAvailable);
    return CONSOLE_SUCCESS;
}

// This is modified for the Wokwi RPi Pico simulator. It works fine
// but that's partially because the serial terminal sends all of the
// characters at a time without losing any of them. What if this function
// wasn't called fast enough?

eConsoleError ConsoleIoReceive(uint8_t *buffer, const uint32_t bufferLength, uint32_t *readLength)
{
    uint32_t i = 0;
    uint8_t rcvd_data;

   while(1)
        {
        UART_read(uart0,&rcvd_data, bufferLength);
            if(rcvd_data =='\r'){
                break;

            }
            else{
                buffer[i++] = rcvd_data;
            }
        }
   UART_write(uart0,&buffer, i);
     *readLength = i;
     return CONSOLE_SUCCESS;

}

eConsoleError ConsoleIoSendString(const char *buffer)
{
   // UART_transmitData(uart1, buffer);
    UART_write(uart0, buffer, strlen(buffer));

    return CONSOLE_SUCCESS;
}

