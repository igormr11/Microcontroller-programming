#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"

uint32_t g_ui32SysClock;
uint32_t  dados[8];
uint32_t samples;
tCANMsgObject sMsgObjectRx;
tCANMsgObject sMsgObjectTx;
uint8_t pui8BufferIn[8];
uint8_t pui8BufferOut[8];

#define ADC_SAMPLE_RATE 40000
volatile uint32_t adc_send_delay = ADC_SAMPLE_RATE;

void adc_interrupt() {
    ADCIntClearEx(ADC0_BASE,  ADCIntStatusEx(ADC0_BASE, true));
    samples = ADCSequenceDataGet(ADC0_BASE, 0, dados);
    if (adc_send_delay) adc_send_delay--;
}

int main(void)
{
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);    // ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));   // ADC
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)); //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0));   //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));  // GPIOs
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));  //

    GPIOPinConfigure(GPIO_PA0_CAN0RX);
    GPIOPinConfigure(GPIO_PA1_CAN0TX);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configuração do Timer que dispara o ADC
    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock / ADC_SAMPLE_RATE - 1);          // Período de Leitura do ADC.
    TimerADCEventSet(TIMER1_BASE, TIMER_ADC_TIMEOUT_A); // Evento que dispara o ADC
    TimerControlTrigger(TIMER1_BASE, TIMER_A, true);
    TimerEnable(TIMER1_BASE, TIMER_A);

    // Configuração do ADC
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH3);
    ADCSequenceEnable(ADC0_BASE, 0);
    ADCIntRegister(ADC0_BASE, 0, adc_interrupt);
    ADCIntEnable(ADC0_BASE, 0);

    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, g_ui32SysClock, 1000000);
    CANRetrySet(CAN0_BASE, false);
    CANEnable(CAN0_BASE);

    sMsgObjectTx.ui32MsgID   = 0x0AD;
    sMsgObjectTx.ui32MsgIDMask = 0x7ff;
    sMsgObjectTx.ui32Flags   = MSG_OBJ_USE_ID_FILTER;
    sMsgObjectTx.ui32MsgLen  = 2;
    sMsgObjectTx.pui8MsgData = pui8BufferOut;

    while (true) {
        if (!adc_send_delay) {
            // mandar
            pui8BufferOut[0] = dados[0] & 0xFF;
            pui8BufferOut[1] = dados[0] >> 8 & 0xFF;

            CANMessageClear(CAN0_BASE, 1);
            CANMessageSet(CAN0_BASE, 1, &sMsgObjectTx, MSG_OBJ_TYPE_RXTX_REMOTE);

            adc_send_delay = ADC_SAMPLE_RATE;
        }
    }

    return 0;
}
