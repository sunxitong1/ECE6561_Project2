/*
 * trajectory_planner.c
 *
 *  Created on: Nov 6, 2016
 *      Author: Joe
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

Semaphore_Handle pathSemHandle;

Void tTrajectoryPlanner(UArg arg0, UArg arg1) {

    if( arg0 == NULL ) {
        System_abort("Sampling semaphore NULL!");
    }

//    sensorSuiteStarted = true;
    pathSemHandle = (Semaphore_Handle) arg0;

    while(1) {
        Semaphore_pend(pathSemHandle, BIOS_WAIT_FOREVER);

        /* Do sampling of sensor stuff */
    }
}
