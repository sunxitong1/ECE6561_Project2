/*
 * motor_control.h
 *
 *  Created on: Nov 3, 2016
 *      Author: Josh
 */

#ifndef COMMS_H_
#define COMMS_H_

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/gates/gateMutex.h>

#include "sensor_suite.h"
#include "motor_control.h"

extern motorControlMsg_t      motorControlMsg;
extern GateMutex_Handle       motorControlMsgMutex;

//extern trajectoryMeasMsg_t    trajectoryMeasMsg; //TODO: WHAT SHOULD THIS BE?
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

inline void motorControlMsgRead( motorControlMsg_t *localMsgObject ) {
	IArg mutexKey;
	mutexKey = GateMutex_enter(motorControlMsgMutex);
	localMsgObject->desiredV = motorControlMsg.desiredV; // Should be 0-100
	localMsgObject->desiredV = motorControlMsg.bias;     // Should be -100 - 100
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

inline void motorMeasurementMsgRead( motorMeasMsg_t *localMsgObject) {
	IArg mutexKey;
	Semaphore_pend(motorSemHandle, BIOS_WAIT_FOREVER);
	mutexKey = GateMutex_enter(motorMeasMsgMutex);
	localMsgObject->leftV = motorMeasMsg.leftV;
	localMsgObject->rightV = motorMeasMsg.rightV;
	GateMutex_leave(motorMeasMsgMutex, mutexKey);
}


#endif /* COMMS_H_ */
