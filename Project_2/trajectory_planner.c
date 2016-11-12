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

#include <timer32.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>

#include "comms.h"
#include "odometryDefs.h"

#ifdef METRICS
#define METRICS_PERIOD  100000
extern uint32_t t0,t1;
extern uint32_t tMeas[1000];
extern int      tIndex;
#endif

#define PI 3.1415


trajectoryMeasMsg_t  localTrajectoryMeasMsg;
motorControlMsg_t    localMotorControlMsg;
IArg mutexKey;

int32_t   xArray[200];
int32_t   yArray[200];
int32_t   tArray[200];
float     dArray[200];

point_t   plot[10];

int8_t   bias = 0;
uint16_t   biasToggle = 0;
uint16_t   velocity = 40;
double   angle = 0.;
double   rad = 0.;

Void tTrajectoryPlanner(UArg arg0, UArg arg1) {

	float i;
	int j = 0;

    if( arg0 == NULL ) {
        System_abort("Sampling semaphore NULL!");
    }

    i = 0.;

    for( j = 0; j<10; j++) {
        plot[j].x = j * 400;
        plot[j].y = 400 * sinf((float) j * 400.);
    }

    j = 0;

    while(1) {
    	if( trajectoryMeasMsgRead( &localTrajectoryMeasMsg ) != TRUE ){
    		; // WE DONE BROKE!
    	}

#ifdef METRICS
        t0 = Timer32_getValue(TIMER32_0_BASE);
#endif

    	if(j < 200 ) {
            xArray[j] = localTrajectoryMeasMsg.xPos;
            yArray[j] = localTrajectoryMeasMsg.yPos;
            tArray[j] = localTrajectoryMeasMsg.distT;
            dArray[j] = localTrajectoryMeasMsg.degPos;
            j++;
    	}
        //bias = sinf(i/2.)*20.;
        //i += 0.15;



        /* Send updated info to motor control */
        motorControlMsgSend( velocity, bias);

#ifdef METRICS
        t1 = Timer32_getValue(TIMER32_0_BASE);
        tMeas[tIndex++]  = t0-t1;;
#endif

    }
}

