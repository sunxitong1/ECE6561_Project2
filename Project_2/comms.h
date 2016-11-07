/*
 * motor_control.h
 *
 *  Created on: Nov 3, 2016
 *      Author: Josh
 */

#ifndef COMMS_H_
#define COMMS_H_

extern uint32_t commDutyValues[2];
extern GateMutex_Struct commMotorMutexStruct;
extern GateMutex_Handle commMotorMutexHandle;

#endif /* COMMS_H_ */