/*
 * trajectory_planner.c
 *
 *  Created on: Nov 6, 2016
 *      Author: Joe
 */

#include <stdbool.h>
#include <math.h>
#include "comms.h"

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

commMotorObject_t localCommMotorObject;
IArg mutexKey;

uint16_t   bias = 50;
uint16_t   biasToggle = 0;
uint16_t   velocity= 20;
double   angle = 0.;
double   rad = 0.;
#define pi 3.1415;

Void tTrajectoryPlanner(UArg arg0, UArg arg1) {


    if( arg0 == NULL ) {
        System_abort("Sampling semaphore NULL!");
    }

    //    sensorSuiteStarted = true;
    pathSemHandle = (Semaphore_Handle) arg0;
    /* Block and receive changes from ? */
    Semaphore_pend(motorSemHandle, BIOS_WAIT_FOREVER);

    /* Update Motor Object */
    mutexKey = GateMutex_enter(commMotorObjectMutex);
    localCommMotorObject = commMotorObject;
    GateMutex_leave(commMotorObjectMutex, mutexKey);

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

        mutexKey = GateMutex_enter(commMotorObjectMutex);
        commMotorObject.desiredV = velocity; // Should be 0-100
        commMotorObject.bias = bias;     // Should be 0-100
        GateMutex_leave(commMotorObjectMutex, mutexKey);
    }
}

