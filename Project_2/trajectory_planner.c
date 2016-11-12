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

trajectoryMeasMsg_t  localTrajectoryMeasMsg;
motorControlMsg_t    localMotorControlMsg;
IArg mutexKey;

int8_t   bias = 0;
uint16_t   biasToggle = 0;
uint16_t   velocity = 60;
double   angle = 0.;
double   rad = 0.;
#define pi 3.1415;

Void tTrajectoryPlanner(UArg arg0, UArg arg1) {

	float i;

    if( arg0 == NULL ) {
        System_abort("Sampling semaphore NULL!");
    }

    i = 0.;

    while(1) {
    	if( trajectoryMeasMsgRead( &localTrajectoryMeasMsg ) != TRUE ){
    		; // WE DONE BROKE!
    	}

        bias = sinf(i/2.)*20.;

        i += 0.15;

        /* Send updated info to motor control */
        motorControlMsgSend( velocity, bias);
    }
}

