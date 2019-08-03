#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

uint32_t g_ui32SysClock;

//Camada de abstração de hardware

bool sensor_open()
   {
    if (GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3))
        return 0;
    else
        return 1;
    }

bool sensor_closed() {
    if (GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_3))
        return 0;
    else
        return 1;
}


bool remote_button() {
    static uint32_t nova = 1;
    uint32_t velha = nova;
    nova = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0);
    return (velha & ~nova);
}


void led_ok(bool on) {
    if (on)
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
    else
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
}

void led_busy(bool on) {
   if (on)
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0);
   else
       GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
}


void led_error(bool on) {
    if (on)
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0 );
    else
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5 );
}

void motor_open() {
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0 );
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

void motor_close() {
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
}

void motor_stop() {
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0 );
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0 );
}



enum g_state {
    G_OPEN,
    G_OPENING_1,
    G_OPENING_2,
    G_CLOSED,
    G_CLOSING_1,
    G_CLOSING_2,
    G_CLOSED_FAKE,
    G_ERROR_TO_OPEN,
    G_ERROR_TO_CLOSED,
} ;

g_state garagem = G_OPEN;

void debounce_int_handler() {
    GPIOIntClear(GPIO_PORTM_BASE, GPIOIntStatus(GPIO_PORTM_BASE, true));     // A função clear limpa os valores retornados pela função status
    GPIOIntClear(GPIO_PORTN_BASE, GPIOIntStatus(GPIO_PORTN_BASE, true));     // A função status retorna o status das interrupções, isto é, a logical OR
    GPIOIntClear(GPIO_PORTJ_BASE, GPIOIntStatus(GPIO_PORTJ_BASE,true));      // das interrupções ativas

    TimerLoadSet (TIMER0_BASE, TIMER_A, g_ui32SysClock/10);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable (TIMER0_BASE, TIMER_A);
}

void start_timer(uint32_t time) {
    TimerLoadSet (TIMER1_BASE, TIMER_A, time*g_ui32SysClock); // Conta tempo em segundos
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable (TIMER1_BASE, TIMER_A);
}

bool timer_done () {
    return TimerIntStatus(TIMER1_BASE, true) & TIMER_TIMA_TIMEOUT;
}

void int_command() {
    TimerDisable (TIMER0_BASE, TIMER_A);
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    switch (garagem) {
        case G_OPEN:
            if (remote_button()) {
              motor_close();
              start_timer(1);
              garagem = G_CLOSING_1;
            } break;

        case G_CLOSING_1:                                               //
            if (remote_button()) {                                      //
                motor_stop();
                garagem = G_CLOSED_FAKE;
            } else if (sensor_closed()) {
                motor_stop();
                garagem = G_CLOSED;
            } else if (!sensor_open()) {
                start_timer(10);
                garagem = G_CLOSING_2;
            } else if (timer_done()) {
                motor_stop();
                garagem = G_ERROR_TO_OPEN;
            } break;


        case G_CLOSING_2:
            if (remote_button()) {
                garagem = G_CLOSED_FAKE;
                motor_stop();
            } else if (sensor_closed()) {
                motor_stop();
                garagem = G_CLOSED;
            } else if (timer_done()) {
                motor_stop();
                garagem = G_ERROR_TO_OPEN;
            } break;

        case G_CLOSED:
            if (remote_button()) {
                motor_open();
                start_timer(1);
                garagem = G_OPENING_1;
            }
            else if (!sensor_closed()) {
                garagem = G_ERROR_TO_OPEN;
            } break;

        case G_CLOSED_FAKE:
            if (remote_button()) {
                motor_open();
                start_timer(1);
                garagem = G_OPENING_1;
            } break;

        case G_OPENING_1:
            if (remote_button()) {
                motor_stop();
                garagem = G_OPEN;
            } else if (sensor_open()) {
                motor_stop();
                garagem = G_OPEN;
            } else if (!sensor_closed()) {
                start_timer(10);
                garagem = G_OPENING_2;
            } else if (timer_done()) {
                motor_stop();
                garagem = G_ERROR_TO_CLOSED;
            } break;

        case G_OPENING_2:
            if (remote_button()) {
                motor_stop();
                garagem = G_OPEN;
            }
            else if (sensor_open()) {
                motor_stop();
                garagem = G_OPEN;
            }
            else if (timer_done()) {
                motor_stop();
                garagem = G_ERROR_TO_CLOSED;
            }
            break;

        case G_ERROR_TO_OPEN:
            if(remote_button())
                 garagem = G_OPEN; break;

        case G_ERROR_TO_CLOSED:
            if(remote_button())
                 garagem = G_CLOSED_FAKE; break;

    }

led_error (garagem == G_ERROR_TO_OPEN || garagem == G_ERROR_TO_CLOSED );
led_ok   (garagem == G_CLOSED || garagem == G_OPEN || garagem == G_CLOSED_FAKE);
led_busy (garagem == G_OPENING_1 || garagem == G_OPENING_2 || garagem == G_CLOSING_1 || garagem == G_CLOSING_2 );

}


int main(void)
{
    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                SYSCTL_CFG_VCO_480), 120000000);


    volatile uint32_t ui32Loop;

    //
    // Habilitando periféricos
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // Espera periféricos estarem prontos
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));


    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5| GPIO_PIN_7);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_3);


    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU); // Configurando portas como pull-up
    GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntRegister(GPIO_PORTJ_BASE, debounce_int_handler);                                           // Registrando interrupções
    GPIOIntRegister(GPIO_PORTM_BASE, debounce_int_handler);
    GPIOIntRegister(GPIO_PORTN_BASE, debounce_int_handler);

    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);                           // Definindo tipo de interrupção
    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);

    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_INT_PIN_0);       // Habilitando pino p/ interrupção, sempre fazer por último
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_INT_PIN_3);
    GPIOIntEnable(GPIO_PORTN_BASE, GPIO_INT_PIN_3);

    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure (TIMER0_BASE, TIMER_CFG_ONE_SHOT);

    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure (TIMER1_BASE, TIMER_CFG_ONE_SHOT);


    TimerIntRegister (TIMER0_BASE, TIMER_A, int_command);
    TimerIntRegister (TIMER1_BASE, TIMER_A, int_command);
    //
    // Loop forever.
    //

    while(1)
    {
    }
}



