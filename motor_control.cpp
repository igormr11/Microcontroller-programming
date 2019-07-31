#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "inc/hw_types.h"
#include <math.h>

#define M_SQRT3     1.73205080756887719000  /* sqrt(3) */
#define M_SQRT2     1.41421356237309504880  /* sqrt(2) */
#define deadtime 420e-9

uint32_t g_ui32SysClock;
uint32_t PWMMAX=10000; //
uint32_t count = 60;
float wref = 377*0.1;
float Mi = 0;
uint32_t int1 = 0;
uint32_t qei = 0;
uint32_t new_pos;
uint32_t old_pos = 0;

float wr = 0;
float wn = 2*M_PI*60;
float Vn = 220;
float E = 100;
float A = Vn/M_SQRT3*M_SQRT2;
float k = 0.5;
float dt = 1.0/12000;
float theta=0;
float V1;
float V2;
float V3;
float D1;
float D2;
float D3;

void RWr_k () {
 //  static uint32_t old_pos = 0;
    new_pos = QEIPositionGet(QEI0_BASE);
    int32_t dif = new_pos - old_pos;

    old_pos = new_pos;
    wr = -dif * (2.*M_PI/5000. *200.);

    float e = wref - wr;
    float ki = 1./(3*3600*200); // 1/3/3600/fsm
    float kp = 1./3600;
    Mi = Mi + ki*e;
    if (Mi < -0.5)
       Mi = -0.5;
    if (Mi > 0.5)
       Mi = 0.5;
    k = Mi + kp*e;
    if (k < -0.5)
        k = -0.5;
    if (k > 0.5)
        k = 0.5;


    qei++;

    //k = 33./180.;
}

void pwmisr ()
{
    if  (PWMGenIntStatus(PWM0_BASE, PWM_GEN_3, 1) & PWM_INT_CNT_ZERO)
    {
        int1++;
        PWMGenIntClear(PWM0_BASE, PWM_GEN_3,PWM_INT_CNT_ZERO);

        theta += wn*k*dt;

        if (theta > M_PI) {
            theta -= 2*M_PI;
        }

        V1 = k * A * sin(theta);
        V2 = k * A * sin(theta+M_PI*2/3);
        V3 = k * A * sin(theta-M_PI*2/3);

        D1 = V1 / E + 0.5;
        D2 = V2 / E + 0.5;
        D3 = V3 / E + 0.5;

        if (D1 < 0) D1 = 0;
        if (D1 > 1) D1 = 1;

        if (D2 < 0) D2 = 0;
        if (D2 > 1) D2 = 1;

        if (D3 < 0) D3 = 0;
        if (D3 > 1) D3 = 1;

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, D1*PWMMAX);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, D2*PWMMAX);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, D3*PWMMAX);

        PWMSyncUpdate(PWM0_BASE, PWM_GEN_3_BIT | PWM_GEN_2_BIT | PWM_GEN_1_BIT);

        count--;
        if (count == 0) {
            RWr_k();
            count = 60;
        }
    }
}

int main(void)
{
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1 | GPIO_PIN_2);

    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPinConfigure(GPIO_PG0_M0PWM4); //
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinConfigure(GPIO_PK5_M0PWM7);
    GPIOPinConfigure(GPIO_PL1_PHA0);
    GPIOPinConfigure(GPIO_PL2_PHB0);

    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN); // Colocar restante das flags
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWMMAX); // 10000 contagens -> freq = 12kHz
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, PWMMAX);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWMMAX);

    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_3, g_ui32SysClock*deadtime, g_ui32SysClock*deadtime); // Definir tempo de delay...
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, g_ui32SysClock*deadtime, g_ui32SysClock*deadtime); // Definir tempo de delay...
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_2, g_ui32SysClock*deadtime, g_ui32SysClock*deadtime); // Definir tempo de delay...

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_SYNC | PWM_GEN_MODE_GEN_NO_SYNC | PWM_GEN_MODE_DBG_RUN | PWM_GEN_MODE_DB_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_SYNC | PWM_GEN_MODE_GEN_NO_SYNC | PWM_GEN_MODE_DBG_RUN | PWM_GEN_MODE_DB_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_SYNC | PWM_GEN_MODE_GEN_NO_SYNC | PWM_GEN_MODE_DBG_RUN | PWM_GEN_MODE_DB_NO_SYNC);

    PWMOutputState(PWM0_BASE,PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMMAX/2);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMMAX/4);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMMAX/8);

    PWMSyncUpdate(PWM0_BASE, PWM_GEN_3_BIT | PWM_GEN_2_BIT | PWM_GEN_1_BIT);

    PWMGenIntRegister(PWM0_BASE, PWM_GEN_3, pwmisr);

    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_3, PWM_INT_CNT_ZERO);

    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_3);

    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B |QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 0xFFFFFFFF);
    QEIEnable(QEI0_BASE);

    while (1) {

    }

    return 0;
}
