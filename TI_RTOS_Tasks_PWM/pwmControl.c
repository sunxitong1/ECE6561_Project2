/*
 * pwmControl.c
 *
 *  Created on: Nov 2, 2016
 *      Author: Josh
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/PWM.h>

/* Example/Board Header files */
#include "Board.h"

/*
 *  ======== pwmLEDFxn ========
 *  Task periodically increments the PWM duty for the on board LED.
 */
Void pwmLEDFxn(UArg arg0, UArg arg1)
{
    PWM_Handle pwm;
    PWM_Params params;
    uint16_t   pwmPeriod = 3000;      // Period and duty in microseconds
    uint16_t   duty = 0;
    uint16_t   dutyInc = 100;

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;

    // Open chosen pwm module
    if(arg1 == 0) {
    	pwm = PWM_open(Board_PWM0, &params);
    }
    else {
    	pwm = PWM_open(Board_PWM1, &params);
    }

    if (pwm == NULL) {
        System_abort("PWM did not open!");
    }
    PWM_start(pwm);

    /* Loop forever incrementing the PWM duty */
    while (1) {
        PWM_setDuty(pwm, duty);

        duty = (duty + dutyInc);
        if (duty == pwmPeriod || (!duty)) {
            dutyInc = - dutyInc;
        }

        Task_sleep((UInt) arg0);
    }
}
