/*
 * trajectory_planner.h
 *
 *  Created on: Nov 6, 2016
 *      Author: Joseph
 */

#ifndef TRAJECTORY_PLANNER_H_
#define TRAJECTORY_PLANNER_H_

typedef struct {
	int32_t xPos;
	int32_t yPos;
	int32_t distT;
	float   degPos;
} trajectoryMeasMsg_t;


Void tTrajectoryPlanner(UArg arg0, UArg arg1);

#endif /* TRAJECTORY_PLANNER_H_ */
