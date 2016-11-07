/*
 * motor_control.h
 *
 *  Created on: Nov 3, 2016
 *      Author: Josh
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

typedef struct {
  uint16_t	desiredV;
  uint8_t	bias;
} commMotorObject;

#define NUM_MOTORS	2

Void tMotorControl(UArg arg0, UArg arg1);

#endif /* MOTOR_CONTROL_H_ */
