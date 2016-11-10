/*
 * sensor_suite.c
 *
 *  Created on: Nov 6, 2016
 *      Author: Josh
 */

#include <stdbool.h>

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

	if( arg0 == NULL ) {
		System_abort("Sampling semaphore NULL!");
	}

	sensorSuiteStarted = true;
	SampSemHandle = (Semaphore_Handle) arg0;

	while(1) {
		Semaphore_pend(SampSemHandle, BIOS_WAIT_FOREVER);

		/* Do sampling of sensor stuff */

		/* Update motor Controller with measurements */
		motorMeasurementMsgSend( enc0TickCount, enc1TickCount);

		/* Update trajectory planner with measurements */


	}
}
