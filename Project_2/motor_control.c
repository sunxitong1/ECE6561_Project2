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

#include <timer32.h>

#include "Board.h"

/* Application Includes*/
#include "motor_control.h"
#include "comms.h"
#include "odometryDefs.h"
#include "project_control.h"


#ifdef METRICS
extern uint32_t mct0,mct1;
extern uint32_t mctMeas[1000];
extern int      tIndex;
#endif


#define PWM_PERIOD_VALUE    3000

#define Ki_num  1.       // integral gain numerator
#define Ki_den  1.       // integral gain denominator
#define Kp      5       // proportional gain numerator

MSP_EXP432P401R_PWMName pwmNames[2] = { Board_PWM0, Board_PWM1 };

/*
*
* tMotorControl
*
* Task for low level control of motors. Task loop blocks on
* reading the motor measurement values to provide feedback
* to the control function. A non-blocking read of the directing
* message from the trajectory planner controls the desired
* velocity and  bias.
*
*/
Void tMotorControl(UArg arg0, UArg arg1) {

	PWM_Handle pwm[NUM_MOTORS];
	PWM_Params pwmParams[NUM_MOTORS];
	uint16_t   duty[NUM_MOTORS];
	int i;

	float e_l, e_r, e_b;              //errors for left vel, right vel, and bias
	float e_b_last, u_b_last;         // save old values for compensation
	float e_l_i, e_r_i;               // integral gain on l/r error
	float u_l, u_r;                   // control commands for left and right wheels
	float u_b;                        // contribution of bias to commands


	motorControlMsg_t localMotorControlMsg;
	motorMeasMsg_t    localMotorMeasMsg;
	
	localMotorControlMsg.desiredV = 1000;
	localMotorControlMsg.bias = 0;

	e_l = 0; e_r = 0; e_b = 0;
	e_l_i = 0; e_r_i = 0;
	u_b_last = 0; e_b_last = 0;
	u_l = 0; u_r = 0;
	u_b = 0;

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
		/* Block and receive changes from Sensor Suite via Motor Measurement Message */
		motorMeasurementMsgRead( &localMotorMeasMsg );

#ifdef METRICS
	    mct0 = Timer32_getValue(TIMER32_0_BASE);
#endif

		motorControlMsgRead( &localMotorControlMsg );
		
		/* Check inputs */		
		if( localMotorControlMsg.bias > 100 ) {localMotorControlMsg.bias = 100;}
		if( localMotorControlMsg.bias < -100 ) {localMotorControlMsg.bias = -100;}
		if( localMotorControlMsg.desiredV > 100 ) {localMotorControlMsg.desiredV = 100;}

		/* Perform controller calculations */
		e_b = localMotorMeasMsg.leftV - localMotorMeasMsg.rightV + localMotorControlMsg.bias;
		u_b = u_b_last + Ki_num*(SAMPLING_PERIOD_US/1000)*(e_b+e_b_last)/Ki_den/2/100;
		e_l = (localMotorControlMsg.desiredV - localMotorMeasMsg.leftV) - u_b;
		e_r = (localMotorControlMsg.desiredV - localMotorMeasMsg.rightV) + u_b;
		e_l_i += e_l;
		e_r_i += e_r;
		u_l = Kp * (e_l + e_l_i/3.);
		u_r = Kp * (e_r + e_r_i/3.);

		// saturate values to +/- 100
		if ( u_l > 100) u_l = 100;
		else if ( u_l < 0) u_l = 0;
		if ( u_r > 100) u_r = 100;
		else if ( u_r < 0) u_r = 0;

		if ( u_b > 100) u_b = 100;
		else if ( u_b < -100) u_b = -100;

		u_b_last = u_b;
		e_b_last = e_b;

		/* Update PWMs */
		for( i = 0; i < NUM_MOTORS; i++ ) {
			if (i == 0) {
				// Set Right motor
				duty[i] = (u_r);
			}
			else {
				// Set Left motor
				duty[i] = (u_l);
			}
			PWM_setDuty(pwm[i], duty[i]);
		}

#ifdef METRICS
        mct1 = Timer32_getValue(TIMER32_0_BASE);
        mctMeas[tIndex++]  = mct0-mct1;
#endif
	}

}
