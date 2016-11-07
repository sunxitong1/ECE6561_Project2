/*
 * motor_control.h
 *
 *  Created on: Nov 3, 2016
 *      Author: Josh
 */

#ifndef COMMS_H_
#define COMMS_H_

#include <ti/sysbios/knl/Semaphore.h>

#include "sensor_suite.h"
#include "motor_control.h"

extern uint32_t           commDutyValues[2];
extern commMotorObject_t  commMotorObject;
extern GateMutex_Handle   commMotorObjectMutex;

extern GateMutex_Handle   commMeasTicksObjectMutex;

extern GateMutex_Handle   commMeasVObjectMutex;

#endif /* COMMS_H_ */