/*
 * trajectory_planner.c
 *
 *  Created on: Nov 6, 2016
 *      Author: Joe
 */

#include <stdbool.h>
#include <math.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>

Semaphore_Handle pathSemHandle;

uint8_t   bias = 50;
uint8_t   biasToggle = 0;
uint8_t   velocity= 50;
double   angle = 0.;
double   rad = 0.;
#define pi 3.1415;

Void tTrajectoryPlanner(UArg arg0, UArg arg1) {


    if( arg0 == NULL ) {
        System_abort("Sampling semaphore NULL!");
    }

    //    sensorSuiteStarted = true;
    pathSemHandle = (Semaphore_Handle) arg0;

    while(1) {
        Semaphore_pend(pathSemHandle, BIOS_WAIT_FOREVER);

        /* Update Bias value */
        if( bias == 0 ) {
            biasToggle = 1;
        }
        if( biasToggle == 0 ) {
            bias = bias-5;
        }
        else {
            bias = bias+5;
        }
        rad = atan(3./4.);
        angle = rad*180.0/pi;
    }
}

