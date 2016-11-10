/*
 * motor_control.h
 *
 *  Created on: Nov 3, 2016
 *      Author: Josh
 */

#ifndef COMMS_H_
#define COMMS_H_

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/gates/gateMutex.h>

#include "sensor_suite.h"
#include "motor_control.h"
#include "trajectory_planner.h"

extern motorControlMsg_t      motorControlMsg;
extern GateMutex_Handle       motorControlMsgMutex;

extern trajectoryMeasMsg_t    trajectoryMeasMsg; //TODO: WHAT SHOULD THIS BE?
extern GateMutex_Handle       trajectoryMsgMutex;
extern Semaphore_Handle       pathSemHandle;

extern motorMeasMsg_t         motorMeasMsg;
extern GateMutex_Handle       motorMeasMsgMutex;
extern Semaphore_Handle       motorSemHandle;

/*
 * Function for updating the motor control variables.
 * No blocking semaphore for this control message.*/
inline void motorControlMsgSend( uint16_t desV, int8_t bias) {
	IArg mutexKey;
	mutexKey = GateMutex_enter(motorControlMsgMutex);
	motorControlMsg.desiredV = desV; // Should be 0-100
	motorControlMsg.bias = bias;     // Should be -100 - 100
	GateMutex_leave(motorControlMsgMutex, mutexKey);
}

inline xdc_Bool motorControlMsgRead( motorControlMsg_t *localMsgObject ) {
	IArg mutexKey;
	xdc_Bool returnVal;
	mutexKey = GateMutex_enter(motorControlMsgMutex);
	localMsgObject->desiredV = motorControlMsg.desiredV; // Should be 0-100
	localMsgObject->bias = motorControlMsg.bias;     // Should be -100 - 100
	GateMutex_leave(motorControlMsgMutex, mutexKey);
}

/*
 * Function for updating the motor with the measurement values
 * Release semaphore and causes motor control task to run.
 * Read blocks on that semaphore.
 */
inline void motorMeasurementMsgSend( uint32_t leftV, uint32_t rightV) {
	IArg mutexKey;
	mutexKey = GateMutex_enter(motorMeasMsgMutex);
	motorMeasMsg.leftV = leftV;
	motorMeasMsg.rightV = rightV;
	GateMutex_leave(motorMeasMsgMutex, mutexKey);
	Semaphore_post(motorSemHandle);
}

inline xdc_Bool motorMeasurementMsgRead( motorMeasMsg_t *localMsgObject) {
	IArg mutexKey;
	xdc_Bool returnVal;
	returnVal = Semaphore_pend(motorSemHandle, BIOS_WAIT_FOREVER);
	mutexKey = GateMutex_enter(motorMeasMsgMutex);
	localMsgObject->leftV = motorMeasMsg.leftV;
	localMsgObject->rightV = motorMeasMsg.rightV;
	GateMutex_leave(motorMeasMsgMutex, mutexKey);
	return returnVal;
}

/* Function for updating the trajectory planner with the
* measured values from the sensory suite.
* Release semaphore and causes trajectory planner task to run.
* Read blocks on that semaphore.
*/
inline void trajectoryMeasMsgSend( int32_t x, int32_t y, int32_t d, float degp) {
	IArg mutexKey;
	mutexKey = GateMutex_enter(trajectoryMsgMutex);
	trajectoryMeasMsg.xPos = x;
	trajectoryMeasMsg.yPos = y;
	trajectoryMeasMsg.distT = d;
	trajectoryMeasMsg.degPos = degp;
	GateMutex_leave(trajectoryMsgMutex, mutexKey);
	Semaphore_post(pathSemHandle);
}

inline xdc_Bool trajectoryMeasMsgRead( trajectoryMeasMsg_t *localMsgObject) {
	IArg mutexKey;
	xdc_Bool returnVal;
	returnVal = Semaphore_pend(pathSemHandle, BIOS_WAIT_FOREVER);
	mutexKey = GateMutex_enter(trajectoryMsgMutex);
	localMsgObject->xPos = trajectoryMeasMsg.xPos;
	localMsgObject->yPos = trajectoryMeasMsg.yPos;
	localMsgObject->distT = trajectoryMeasMsg.distT;
	localMsgObject->degPos = trajectoryMeasMsg.degPos;
	GateMutex_leave(trajectoryMsgMutex, mutexKey);
	return returnVal;
}

#endif /* COMMS_H_ */
