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

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

/* Board Header file */
#include "Board.h"

/* Application Includes */
#include "motor_control.h"
#include "sensor_suite.h"

#define SAMPLING_RATE_US	50000
#define PATHING_PERIOD_US	500000

#define HEARTBEAT_TASK_PRIO      3
#define MOTORCONTROL_TASK_PRIO   3
#define SENSORSUITE_TASK_PRIO    3

#define TASKSTACKSIZE            512


Task_Struct task0Struct;
Task_Struct task1Struct;
Task_Struct task2Struct;
Task_Struct task3Struct;
Char task0Stack[TASKSTACKSIZE];
Char task1Stack[TASKSTACKSIZE];
Char task2Stack[TASKSTACKSIZE];
Char task3Stack[TASKSTACKSIZE];

Semaphore_Struct sem0Struct;
Semaphore_Handle sem0Handle;
Semaphore_Struct SampSemStruct;
Semaphore_Handle SampSemHandle;
Semaphore_Struct pathSemStruct;
Semaphore_Handle pathSemHandle;


Clock_Struct clk0Struct;
Clock_Struct clk1Struct;


Void heartBeatFxn(UArg arg0, UArg arg1);
Void clk0Fxn(UArg arg0);
Void clk1Fxn(UArg arg0);


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

    /* Install Encoder callbacks */
    GPIO_setCallback(Motor_Encoder_0, motorEncIntHandler0);
    GPIO_enableInt(Motor_Encoder_0);
    GPIO_setCallback(Motor_Encoder_1, motorEncIntHandler1);
    GPIO_enableInt(Motor_Encoder_1);

    /* Construct a Semaphore object to be used as a resource lock, initial count 0 */
	Semaphore_Params_init(&semParams);
	Semaphore_construct(&sem0Struct, 0, &semParams);
	sem0Handle = Semaphore_handle(&sem0Struct);

	/* Construct a Semaphore object to be used as a resource lock, initial count 0 */
	Semaphore_Params_init(&semParams);
	Semaphore_construct(&SampSemStruct, 0, &semParams);
	SampSemHandle = Semaphore_handle(&SampSemStruct);

	/* Construct a Semaphore object to be used as a resource lock, initial count 0 */
	Semaphore_Params_init(&semParams);
	Semaphore_construct(&pathSemStruct, 0, &semParams);
	pathSemHandle = Semaphore_handle(&pathSemStruct);

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    taskParams.priority = HEARTBEAT_TASK_PRIO;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);

    /* Construct motor control Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = (UArg) sem0Handle;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task1Stack;
    taskParams.priority = MOTORCONTROL_TASK_PRIO;
    Task_construct(&task1Struct, (Task_FuncPtr)tMotorControl, &taskParams, NULL);

    /* Construct sensor suite Task  thread */
	Task_Params_init(&taskParams);
	taskParams.arg0 = (UArg) SampSemHandle;
	taskParams.stackSize = TASKSTACKSIZE;
	taskParams.stack = &task2Stack;
    taskParams.priority = SENSORSUITE_TASK_PRIO;
    Task_construct(&task2Struct, (Task_FuncPtr)tSensorSuite, &taskParams, NULL);

	/* Construct clock for sampling period release */
	Clock_Params_init(&clkParams);
	clkParams.period = SAMPLING_RATE_US/Clock_tickPeriod;
	clkParams.startFlag = TRUE;
	Clock_construct(&clk0Struct, (Clock_FuncPtr)clk0Fxn,
			SAMPLING_RATE_US/Clock_tickPeriod, &clkParams);

	/* Construct clock for sampling period release */
	Clock_Params_init(&clkParams);
	clkParams.period = PATHING_PERIOD_US/Clock_tickPeriod;
	clkParams.startFlag = TRUE;
	Clock_construct(&clk1Struct, (Clock_FuncPtr)clk1Fxn,
			PATHING_PERIOD_US/Clock_tickPeriod, &clkParams);

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
        Semaphore_post(sem0Handle);
    }
}

/*
 *  ======== clk0Fxn =======
 */
Void clk0Fxn(UArg arg0)
{

    Semaphore_post(SampSemHandle);

}

/*
 *  ======== clk1Fxn =======
 */
Void clk1Fxn(UArg arg0)
{

    Semaphore_post(pathSemHandle);

}
