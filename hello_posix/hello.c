/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
 *  ======== hello.c ========
 */

/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/posix/pthread.h>
#include <ti/sysbios/posix/semaphore.h>
#include <ti/sysbios/posix/sched.h>
#include <ti/sysbios/posix/unistd.h>

/* Example/Board Header files */
#include "Board.h"

sem_t waitSemaphore;
pthread_barrier_t barrier;  // for synchronizing threads

static void * foo( void * arg) {

	while(1) {
		System_printf("hello again\n");
		System_flush();
		pthread_barrier_wait( &barrier );
	}
}

static void * bar( void * arg) {

	while(1) {
		sleep(1);
		pthread_barrier_wait( &barrier );
	}
}

/*
 *  ======== main ========
 */
Int main()
{

	pthread_t threadID1, threadID2;

    /* Call board init functions */
    Board_initGeneral();

    sem_init(&waitSemaphore, 0, 0);
    pthread_barrier_init( &barrier, NULL, 2);

    System_printf("hello world\n");
    System_flush();

    pthread_create(&threadID1, NULL, foo, NULL);
    pthread_create(&threadID2, NULL, bar, NULL);

    BIOS_start();

    // DO NOT RUN pthread_join FROM MAIN
    //pthread_join( threadID1, NULL );

}
