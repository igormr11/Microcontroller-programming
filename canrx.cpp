#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/can.h"

uint32_t g_ui32SysClock;
tCANMsgObject sMsgObjectRx;
tCANMsgObject sMsgObjectTx;
uint8_t pui8BufferIn[8];
uint8_t pui8BufferOut[8];

void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    //
    // Configure GPIO Pins for UART mode.
    //

    MAP_GPIOPinConfigure(GPIO_PD4_U2RX);
    MAP_GPIOPinConfigure(GPIO_PD5_U2TX);
    MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(2, 115200, g_ui32SysClock);
}

int main(void) {

    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);

    ConfigureUART();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    GPIOPinConfigure(GPIO_PA1_CAN0TX);
    GPIOPinConfigure(GPIO_PA0_CAN0RX);

    GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0);
    GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_1);

    CANInit(CAN0_BASE);

    CANBitRateSet(CAN0_BASE, g_ui32SysClock, 1000000);

    sMsgObjectRx.ui32MsgID = 0x0AD;
    sMsgObjectRx.ui32MsgIDMask = 0x7ff;
    sMsgObjectRx.ui32MsgLen = 8;
    sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER;
    sMsgObjectRx.pui8MsgData = pui8BufferIn;
    CANMessageSet(CAN0_BASE, 1, &sMsgObjectRx, MSG_OBJ_TYPE_RX);

    sMsgObjectTx.ui32MsgID = 0x0AD;
    sMsgObjectTx.ui32MsgIDMask = 0x7ff;
    sMsgObjectTx.ui32MsgLen = 0;
    sMsgObjectTx.ui32Flags = MSG_OBJ_USE_ID_FILTER;
    sMsgObjectTx.pui8MsgData = 0;


    CANEnable(CAN0_BASE);

    UARTprintf("Aguardando...\n");

    int contador = 1;

    while (1) {
        if (!--contador) {
            CANMessageSet(CAN0_BASE, 2, &sMsgObjectTx, MSG_OBJ_TYPE_TX_REMOTE);
            contador = 1e6;
        }

        if (CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT) & 0x00000001) {
           CANMessageGet(CAN0_BASE, 1, &sMsgObjectRx, true);
           UARTprintf("%08x",sMsgObjectRx.ui32MsgID);
           for (int i = 0; i < sMsgObjectRx.ui32MsgLen; i++)
               UARTprintf(" %02x", pui8BufferIn[i]);
           UARTprintf(" \n");
       }
    }
	return 0;
}

