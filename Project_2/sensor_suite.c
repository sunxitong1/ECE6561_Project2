/*
 * sensor_suite.c
 *
 *  Created on: Nov 6, 2016
 *      Author: Josh
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

#include <timer32.h>

/* Board Header file */
#include "Board.h"

#include "sensor_suite.h"
#include "comms.h"
#include "odometryDefs.h"

#ifdef METRICS
#define METRICS_PERIOD  100000
extern uint32_t t0,t1;
extern uint32_t tMeas[1000];
extern int      tIndex;
#endif


#define PI 3.1415
#define NUM_ENCODERS     2

bool		sensorSuiteStarted = false;
uint32_t	encTickCount[NUM_ENCODERS] = { 0, 0 };

Semaphore_Handle SampSemHandle;


void motorEncIntHandler0(unsigned int index)
{
	if(sensorSuiteStarted) {
		encTickCount[0]++;
	}
}

void motorEncIntHandler1(unsigned int index)
{
	if(sensorSuiteStarted) {
		encTickCount[1]++;
	}

}

uint32_t getEncTickCount(int encNum) {
	if( encNum < NUM_ENCODERS) {
		return encTickCount[encNum];
	}
	else {
		return 0;
	}
}

////////////////////////////////////////////////////////////////////////////////
// GetVel returns the current velocity using advanced mathematics.
int32_t GetVel( float angle_old, float angle, int32_t vel_old)
{
//     int vel;
     return (A*200*(angle-angle_old)-(A-200)*vel_old)/(A+200);
//     return vel;

}

Void tSensorSuite(UArg arg0, UArg arg1) {

	// variables
	int32_t CountL, CountR;           // motor rotation counts
	int32_t pCountL, pCountR;         // previous counts
	int32_t DistL, DistR, DistC;      // distance variables
	int32_t DistT;
	int32_t Xpos, Ypos;                // X and Y position
	float   DegPos, RadPos;            // Angle Position (in degrees / radians)

	float vel_left, vel_right;        // left, right velocities

	int i = 0;

	if( arg0 == NULL ) {
		System_abort("Sampling semaphore NULL!");
	}

	sensorSuiteStarted = true;
	SampSemHandle = (Semaphore_Handle) arg0;

	/* Initialize all the variables! */
	CountL = 0; CountR = 0;
	pCountL = 0; pCountR = 0;
	DistL = 0; DistR = 0; DistC = 0;
	Xpos = 0; Ypos = 0; DistT = 0;
	DegPos = 0.0; RadPos = 0.0;

	while(1) {
		Semaphore_pend(SampSemHandle, BIOS_WAIT_FOREVER);

#ifdef METRICS
        t0 = Timer32_getValue(TIMER32_0_BASE);
#endif /* METRICS */

		/* Do sampling of sensor stuff */
		pCountL = CountL;
		pCountR = CountR;
		CountL = getEncTickCount(0);
		CountR = getEncTickCount(1);

		/* Estimate distance traveled per wheel */
		DistL = (CountL-pCountL) * WHEELCIRC_MM / 36.;  // DistL mm*10 (tenths of millimeters)
		DistR = (CountR-pCountR) * WHEELCIRC_MM / 36.;  // DistR mm*10
		DistC = (DistL+DistR)/2;                       // DistC mm*10

		/* Skip everything if distance traveled sample is bad */
		if( DistL > TOO_MUCH_DISTANCE || DistL > TOO_MUCH_DISTANCE ) continue;

		/* Calculate the angle in radians? */
		RadPos = (DistR - DistL); // /(WHEELBASE_MM); // Rad xWHEELBASE TODO: WAT

		/* Calculate the angle in degrees through magic */
		//DegPos = fmodf((RadPos * 573.0f) / (100.0f * WHEELBASE_MM), 360.0f);  // Deg
		DegPos += 180.0*RadPos/(PI*WHEELBASE_MM*10.);
		DegPos = fmodf(DegPos, 360.0);
		if(DegPos<0) DegPos += 360.0f;

		DistT += DistC;                                // Total distance

		Xpos += DistC * cosf(DegPos*PI/180); //10.0f;               // Xpos mm*10
		Ypos += DistC * sinf(DegPos*PI/180);//10.0f;               // Ypos mm*10

		/* Get velocity in mm/s, (.010 m) / us  */
		vel_left = (float) DistL / ( SAMPLING_PERIOD_US / 100000. );
		vel_right = (float) DistR / ( SAMPLING_PERIOD_US / 100000. );

		/* Update motor Controller with measurements */
#ifndef METRICS
		motorMeasurementMsgSend( vel_left, vel_right);
#endif

		/* Update trajectory planner with measurements  every 10 loops*/
		++i;
		if( i == 5 ) {
			i = 0;
#ifndef METRICS
			trajectoryMeasMsgSend( Xpos, Ypos, DistT, DegPos );
#endif
		}
#ifdef METRICS
        t1 = Timer32_getValue(TIMER32_0_BASE);
        tMeas[tIndex++]  = t0-t1;;
#endif


	}
}
