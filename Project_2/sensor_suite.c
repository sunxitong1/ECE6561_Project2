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

/* Board Header file */
#include "Board.h"

#include "sensor_suite.h"
#include "comms.h"
#include "odometryDefs.h"

bool		sensorSuiteStarted = false;
uint32_t	enc0TickCount = 0;
uint32_t	enc1TickCount = 0;

Semaphore_Handle SampSemHandle;


void motorEncIntHandler0(unsigned int index)
{
	if(sensorSuiteStarted) {
		enc0TickCount++;
	}
}

void motorEncIntHandler1(unsigned int index)
{
	if(sensorSuiteStarted) {
		enc1TickCount++;
	}

}

Void tSensorSuite(UArg arg0, UArg arg1) {

	// variables
	int32_t CountL, CountR;           // motor rotation counts
	int32_t pCountL, pCountR;         // previous counts
	int32_t DistL, DistR, DistC;      // distance variables
	int32_t DistT;
	int32_t Xpos, Ypos;                // X and Y position
	float   DegPos, RadPos;            // Angle Position (in degrees / radians)

	//int32_t e_l, e_r, e_b;              //errors for left vel, right vel, and bias
//	int32_t e_b_last, u_b_last;         // save old values for compensation
//	int32_t e_l_i, e_r_i;               // integral gain on l/r error
//	int32_t vel_left, vel_right;        // left, right, and reference velocities
//	int32_t u_l, u_r;                   // control commands for left and right wheels
//	int32_t u_b;                        // contribution of bias to commands
//	int32_t angle_left_old,angle_right_old;
//	int32_t angle_left, angle_right;
//	int32_t vel_left_old, vel_right_old;

	if( arg0 == NULL ) {
		System_abort("Sampling semaphore NULL!");
	}

	sensorSuiteStarted = true;
	SampSemHandle = (Semaphore_Handle) arg0;

	/* Initialize all the variables! */
	CountL = 0; CountR = 0;
	pCountL = 0; pCountR = 0;
	DistL = 0; DistR = 0; DistC = 0;



	while(1) {
		Semaphore_pend(SampSemHandle, BIOS_WAIT_FOREVER);

		/* Do sampling of sensor stuff */
		pCountL = CountL;
		pCountR = CountR;
		CountL = enc0TickCount;
		CountR = enc1TickCount;


		DistL = (CountL-pCountL) * WHEELCIRC_MM / 36;  // DistL mm*10 (tenths of millimeters)
		DistR = (CountR-pCountR) * WHEELCIRC_MM / 36;  // DistR mm*10
		DistC = (DistL+DistR)/2;                       // DistC mm*10

		RadPos += (DistL - DistR); // /(WHEELBASE_MM); // Rad xWHEELBASE TODO: WAT

		DegPos = fmodf((RadPos * 573.0f) / (100.0f * WHEELBASE_MM), 360.0f);  // Deg
		if(DegPos<0) DegPos += 360.0f;

		DistT += DistC;                                // Total distance

		Xpos += DistC * cosf(DegPos) /10.0f;               // Xpos mm*10
		Ypos += DistC * sinf(DegPos) /10.0f;               // Ypos mm*10

		/* Update motor Controller with measurements */
		motorMeasurementMsgSend( 0, 0);

		/* Update trajectory planner with measurements */
		//trajectoryMeasMsgSend( Xpos, Ypos, DistT, DegPos );


	}
}
