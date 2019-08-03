#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"

uint32_t f_int = 0;
uint32_t g_ui32SysClock;
uint32_t p_int = 0;
uint32_t edges;
uint32_t time;

void frequency_meas(){
    TimerIntClear(TIMER1_BASE, TimerIntStatus(TIMER1_BASE, true));
    static uint32_t val2 = 0;
    uint32_t val1 = val2;
    val2 = TimerValueGet(TIMER0_BASE, TIMER_A); 
    edges = val1-val2;
    edges&=0x00FFFFFF;
}



void period_meas(){
    TimerIntClear(TIMER2_BASE, TimerIntStatus(TIMER2_BASE, true));
    static uint32_t t2 = 0;
    uint32_t t1 = t2;
    t2 = TimerValueGet(TIMER2_BASE,TIMER_A);
    time = t1-t2;
    time&=0x00FFFFFF;
}

int main(void)
{
    //
    // Inicia o clock a 120 MHz
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                    SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                    SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    GPIOPinConfigure(GPIO_PD0_T0CCP0);
    GPIOPinConfigure(GPIO_PA4_T2CCP0);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6); // Usado p/teste

    GPIOPinTypeTimer(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_INT_PIN_0);

    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_SYSTEM);
    TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);

    TimerPrescaleSet (TIMER0_BASE, TIMER_A, 0x00FF);  
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0XFFFF);
    TimerPrescaleMatchSet(TIMER0_BASE, TIMER_A, 0);
    TimerMatchSet(TIMER0_BASE, TIMER_A, 0x00);

    TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock);    // Contagem de 1 segundo

    TimerPrescaleSet (TIMER2_BASE, TIMER_A, 0x00FF);  
    TimerLoadSet(TIMER2_BASE, TIMER_A, 0XFFFF);
    TimerPrescaleMatchSet(TIMER2_BASE, TIMER_A, 0);
    TimerMatchSet(TIMER2_BASE, TIMER_A, 0x00);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT); 
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);                           
    TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME);  

    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

    TimerIntRegister(TIMER1_BASE, TIMER_A, frequency_meas);
    TimerIntRegister(TIMER2_BASE, TIMER_A, period_meas);

    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER2_BASE,TIMER_CAPA_EVENT);

    TimerEnable (TIMER0_BASE, TIMER_A);
    TimerEnable (TIMER1_BASE, TIMER_A);
    TimerEnable (TIMER2_BASE, TIMER_A);

    while (1) {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);  
        SysCtlDelay(24000);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
        SysCtlDelay(24000);
    }

    return 0;
}
