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

#define PWM_PERIOD_VALUE    3000

MSP_EXP432P401R_PWMName pwmNames[2] = { Board_PWM0, Board_PWM1 };

/*
*
* tMotorControl
*
* 
*
*/
Void tMotorControl(UArg arg0, UArg arg1) {

	PWM_Handle pwm[NUM_MOTORS];
	PWM_Params pwmParams[NUM_MOTORS];
	uint16_t   duty[NUM_MOTORS];
	int i;

	motorControlMsg_t localMotorControlMsg;
	
	localMotorControlMsg.desiredV = 1000;
	localMotorControlMsg.bias = 0;

	/* Initialize pwms */
	for( i = 0; i < NUM_MOTORS; i++ ) {
		duty[i] = 0;

		PWM_Params_init(&(pwmParams[i]));
		pwmParams[i].dutyUnits = PWM_DUTY_US;
		pwmParams[i].dutyValue = 0;
		pwmParams[i].periodUnits = PWM_PERIOD_US;
		pwmParams[i].periodValue = PWM_PERIOD_VALUE;

		pwm[i] = PWM_open(pwmNames[i], &(pwmParams[i]));

		if (pwm[i] == NULL) {
			System_abort("PWM did not open!");
		}
		PWM_start(pwm[i]);
	}

	while (1) {
		/* Block and receive changes from ? */
		motorControlMsgRead( &localMotorControlMsg );
		
		/* Check inputs */		
		if( localMotorControlMsg.bias > 100 ) {localMotorControlMsg.bias = 100;}
		if( localMotorControlMsg.bias < -100 ) {localMotorControlMsg.bias = -100;}
		if( localMotorControlMsg.desiredV > 100 ) {localMotorControlMsg.desiredV = 100;}

		/* Update PWMs */
		/* Desired V is currently in linear percentage of PWM output */
		/* Bias proportion is (100+bias)/2 or (100-bias)/2 */
		for( i = 0; i < NUM_MOTORS; i++ ) {
			if (i == 0) {
				duty[i] = ((localMotorControlMsg.desiredV * PWM_PERIOD_VALUE/100)*(100+localMotorControlMsg.bias))/200;
			}
			else {
				duty[i] = ((localMotorControlMsg.desiredV * PWM_PERIOD_VALUE/100)*(100-localMotorControlMsg.bias))/200;
			}
			PWM_setDuty(pwm[i], duty[i]);
		}
	}

}

/* Example for updating the motor values:
	
	#include "comms.h"

	mutexKey = GateMutex_enter(commMotorObjectMutex);
	commMotorObject.desiredV = 50; // Should be 0-100
	commMotorObject.bias = 50;     // Should be -100 - 100
	GateMutex_leave(commMotorObjectMutex, mutexKey);
*/
