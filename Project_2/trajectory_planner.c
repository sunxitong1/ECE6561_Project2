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
//trajectoryMeasMsg_t  logarray[200];
int32_t   xArray[200];
int32_t   yArray[200];
int32_t   tArray[200];
float     dArray[200];
IArg mutexKey;

int8_t   bias = 0;
uint16_t   biasToggle = 0;
uint16_t   velocity = 40;
double   angle = 0.;
double   rad = 0.;
#define pi 3.1415

Void tTrajectoryPlanner(UArg arg0, UArg arg1) {

	float i;
	int j = 0;

    if( arg0 == NULL ) {
        System_abort("Sampling semaphore NULL!");
    }

    i = 0.;

    while(1) {
    	if( trajectoryMeasMsgRead( &localTrajectoryMeasMsg ) != TRUE ){
    		; // WE DONE BROKE!
    	}
    	if(j < 200 ) {
            xArray[j] = localTrajectoryMeasMsg.xPos;
            yArray[j] = localTrajectoryMeasMsg.yPos;
            tArray[j] = localTrajectoryMeasMsg.distT;
            dArray[j] = localTrajectoryMeasMsg.degPos;
            j++;
    	}
        bias = sinf(i/2.)*20.;
        i += 0.15;

        /* Send updated info to motor control */
        motorControlMsgSend( velocity, bias);
    }
}

