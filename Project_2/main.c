/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/gates/gateMutex.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

#include <timer32.h>

/* Board Header file */
#include "Board.h"

/* Application Includes */
#include "motor_control.h"
#include "sensor_suite.h"
#include "trajectory_planner.h"
#include "odometryDefs.h"
#include "project_control.h"

#define HEARTBEAT_TASK_PRIO      3
#define MOTORCONTROL_TASK_PRIO   3
#define SENSORSUITE_TASK_PRIO    3

#define TASKSTACKSIZE            512
#define HEARTBEATSTACKSIZE       512
#define SENSORSSTACKSIZE         1536
#define MOTORCONTROLSTACKSIZE    1536
#define TRAJECTORYSTACKSIZE      512



Task_Struct task0Struct;
Task_Struct task1Struct;
Task_Struct task2Struct;
Task_Struct task3Struct;
Char task0Stack[HEARTBEATSTACKSIZE];
Char task1Stack[SENSORSSTACKSIZE];
Char task2Stack[MOTORCONTROLSTACKSIZE];
Char task3Stack[TRAJECTORYSTACKSIZE];

Semaphore_Struct motorSemStruct;
Semaphore_Handle motorSemHandle;
Semaphore_Struct SampSemStruct;
Semaphore_Handle SampSemHandle;
Semaphore_Struct pathSemStruct;
Semaphore_Handle pathSemHandle;

Clock_Struct clk0Struct;
Clock_Struct clk1Struct;

Void heartBeatFxn(UArg arg0, UArg arg1);
Void clk0Fxn(UArg arg0);
Void clk1Fxn(UArg arg0);

/* comms variables */
motorControlMsg_t      motorControlMsg;
GateMutex_Handle       motorControlMsgMutex;
GateMutex_Struct       motorControlMsgMutexStruct;


trajectoryMeasMsg_t    trajectoryMeasMsg;
GateMutex_Handle       trajectoryMsgMutex;
GateMutex_Struct       trajectoryMsgMutexStruct;
Semaphore_Handle       pathSemHandle;

motorMeasMsg_t         motorMeasMsg;
GateMutex_Handle       motorMeasMsgMutex;
GateMutex_Struct       motorMeasMsgMutexStruct;
Semaphore_Handle       motorSemHandle;

#ifdef METRICS
uint32_t t0,t1;
uint32_t tMeas[1000];
int      tIndex = 0;
#endif

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;
    Semaphore_Params semParams;
    Clock_Params clkParams;


    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initPWM();

    Timer32_initModule((uint32_t) TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_FREE_RUN_MODE);
    Timer32_startTimer((uint32_t) TIMER32_0_BASE, false);


    /* Install Encoder callbacks */
    GPIO_setCallback(Motor_Encoder_0, motorEncIntHandler0);
    GPIO_enableInt(Motor_Encoder_0);
    GPIO_setCallback(Motor_Encoder_1, motorEncIntHandler1);
    GPIO_enableInt(Motor_Encoder_1);

    /* Construct a Semaphore object to be used as a resource lock, initial count 0 */
	Semaphore_Params_init(&semParams);
	Semaphore_construct(&motorSemStruct, 0, &semParams);
	motorSemHandle = Semaphore_handle(&motorSemStruct);

	/* Construct a Semaphore object to be used as a resource lock, initial count 0 */
	Semaphore_Params_init(&semParams);
	Semaphore_construct(&SampSemStruct, 0, &semParams);
	SampSemHandle = Semaphore_handle(&SampSemStruct);

	/* Construct a Semaphore object to be used as a resource lock, initial count 0 */
	Semaphore_Params_init(&semParams);
	Semaphore_construct(&pathSemStruct, 0, &semParams);
	pathSemHandle = Semaphore_handle(&pathSemStruct);
	
	/* Construct Mutexes for comms structures */
	GateMutex_construct(&motorControlMsgMutexStruct, NULL);
	motorControlMsgMutex = GateMutex_handle(&motorControlMsgMutexStruct);
	GateMutex_construct(&trajectoryMsgMutexStruct, NULL);
	trajectoryMsgMutex = GateMutex_handle(&trajectoryMsgMutexStruct);
	GateMutex_construct(&motorMeasMsgMutexStruct, NULL);
	motorMeasMsgMutex = GateMutex_handle(&motorMeasMsgMutexStruct);

    /* Construct heartBeat Task  thread  (BUT NOT IF DOING METRICS)*/
#ifndef METRICS
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000;
    taskParams.stackSize = HEARTBEATSTACKSIZE;
    taskParams.stack = &task0Stack;
    taskParams.priority = HEARTBEAT_TASK_PRIO;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);
#endif

    /* Construct motor control Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = (UArg) motorSemHandle;
    taskParams.stackSize = MOTORCONTROLSTACKSIZE;
    taskParams.stack = &task1Stack;
    taskParams.priority = MOTORCONTROL_TASK_PRIO;
    Task_construct(&task1Struct, (Task_FuncPtr)tMotorControl, &taskParams, NULL);

    /* Construct sensor suite Task  thread */
	Task_Params_init(&taskParams);
	taskParams.arg0 = (UArg) SampSemHandle;
	taskParams.stackSize = SENSORSSTACKSIZE;
	taskParams.stack = &task2Stack;
    taskParams.priority = SENSORSUITE_TASK_PRIO;
    Task_construct(&task2Struct, (Task_FuncPtr)tSensorSuite, &taskParams, NULL);

    /* Construct trajectory planner Task  thread */
	Task_Params_init(&taskParams);
	taskParams.arg0 = (UArg) pathSemHandle;
	taskParams.stackSize = TRAJECTORYSTACKSIZE;
	taskParams.stack = &task3Stack;
	Task_construct(&task3Struct, (Task_FuncPtr)tTrajectoryPlanner, &taskParams, NULL);

	/* Construct clock for sampling period release */
	Clock_Params_init(&clkParams);
	clkParams.period = SAMPLING_PERIOD_US/Clock_tickPeriod;
	clkParams.startFlag = TRUE;
	Clock_construct(&clk0Struct, (Clock_FuncPtr)clk0Fxn,
			SAMPLING_PERIOD_US/Clock_tickPeriod, &clkParams);

	/* Construct clock for sampling period release */
#ifdef METRICS
	Clock_Params_init(&clkParams);
	clkParams.period = METRICS_PERIOD/Clock_tickPeriod;
	clkParams.startFlag = TRUE;
	Clock_construct(&clk1Struct, (Clock_FuncPtr)clk1Fxn,
			METRICS_PERIOD/Clock_tickPeriod, &clkParams);
#endif

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();


    /* Start BIOS */
    BIOS_start();

    return (0);
}

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void heartBeatFxn(UArg arg0, UArg arg1)
{

    while (1) {
        Task_sleep((UInt)arg0);
        
		GPIO_toggle(Board_LED0);

    }
}

/*
 *  ======== clk0Fxn =======
 */
Void clk0Fxn(UArg arg0)
{
#ifndef METRICS
    Semaphore_post(SampSemHandle);
#endif

}

/*
 *  ======== clk1Fxn =======
 */
Void clk1Fxn(UArg arg0)
{
#ifdef METRICS
    //Semaphore_post( motorSemHandle );
    Semaphore_post( pathSemHandle );
    //Semaphore_post(SampSemHandle);
#endif

}
