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

#define PLOTCOUNT 12

trajectoryMeasMsg_t  localTrajectoryMeasMsg;
motorControlMsg_t    localMotorControlMsg;
IArg mutexKey;

int32_t   xArray[200];
int32_t   yArray[200];
int32_t   tArray[200];
float     dArray[200];

point_t   plot[PLOTCOUNT];

int8_t   bias = 0;
uint16_t   biasToggle = 0;
uint16_t   velocity = 40;
double   angle = 0.;
double   rad = 0.;

Void tTrajectoryPlanner(UArg arg0, UArg arg1) {

	float i;
	int j;
	int plotIndex;

	int32_t deltaX, deltaY, deltaD;
	float    deltaDeg, deltaDegDeg;

    if( arg0 == NULL ) {
        System_abort("Sampling semaphore NULL!");
    }



    for( j = 0; j<PLOTCOUNT; j++) {
        plot[j].x = j * 4000 * PI / 10;
        plot[j].y = 4000 * sinf((float) j * PI/10.);
    }

    i = 0.;
    j = 0;
    plotIndex = 0;

    while(1) {
    	if( trajectoryMeasMsgRead( &localTrajectoryMeasMsg ) != TRUE ){
    		; // WE DONE BROKE!
    	}

    	deltaX = plot[plotIndex].x - localTrajectoryMeasMsg.xPos;
    	deltaY = plot[plotIndex].y - localTrajectoryMeasMsg.yPos;

    	if(deltaX < 100 && deltaY < 100 && plotIndex < PLOTCOUNT) {
    	    plotIndex++;
    	}
    	else if ( plotIndex == PLOTCOUNT) {
    	    ; // SHUT HER DOWN;
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


    	deltaD = sqrtf(powf((float) deltaX, 2.) + powf((float) deltaY, 2.));
    	deltaDeg = atanf((float) deltaX / (float) deltaY)*180 / PI;

    	deltaDegDeg = fmodf(deltaDeg - localTrajectoryMeasMsg.degPos, 360.);


    	if ( deltaDegDeg < 180 ) {
    	    bias = -1 * (deltaDegDeg / 180) * 100;
    	    velocity = ((deltaDegDeg / 180) * 20) + 10;
    	}
    	else {
    	    bias =  ((deltaDegDeg-180) / 180) * 10;
    	    velocity = (((deltaDegDeg-180) / 180) * 20) + 10;
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

