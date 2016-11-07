/*
 * motor_control.c
 *
 *  Created on: Nov 3, 2016
 *      Author: Josh
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drivers/PWM.h>

#include "Board.h"

#include "motor_control.h"
#include "comms.h"

MSP_EXP432P401R_PWMName pwmNames[2] = { Board_PWM0, Board_PWM1 };


Void tMotorControl(UArg arg0, UArg arg1) {

	PWM_Handle pwm[NUM_MOTORS];
	PWM_Params pwmParams[NUM_MOTORS];
	uint16_t   duty[NUM_MOTORS];

	Semaphore_Handle semHandle;
	commMotorObject_t localCommMotorObject;
	IArg mutexKey;
	
	semHandle = (Semaphore_Handle) arg0;
	localCommMotorObject.desiredV = 0;
	localCommMotorObject.bias = 50;

	/* Initialize pwms */
	for( i = 0; i < NUM_MOTORS; i++ ) {
		duty[i] = 0;

		PWM_Params_init(&(pwmParams[i]));
		pwmParams[i].dutyUnits = PWM_DUTY_US;
		pwmParams[i].dutyValue = 0;
		pwmParams[i].periodUnits = PWM_PERIOD_US;
		pwmParams[i].periodValue = 3000;

		pwm[i] = PWM_open(pwmNames[i], &(pwmParams[i]));

		if (pwm[i] == NULL) {
			System_abort("PWM did not open!");
		}
		PWM_start(pwm[i]);
	}

	while (1) {
		/* Block and receive changes from ? */
		Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);

		/* Update PWMs */
		mutexKey = GateMutex_enter(commMotorObjectMutex);
		localCommMotorObject = commMotorObject;
		for( i = 0; i < NUM_MOTORS; i++ ) {
			duty[i] = commDutyValues[i];
			PWM_setDuty(pwm[i], duty[i]);
		}
		GateMutex_leave(commMotorObjectMutex, mutexKey);
	}

}

