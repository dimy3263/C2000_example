//###########################################################################
//
// FILE:   ipc_ex1_gpio_toggle_cpu1.c
//
// TITLE:  GPIO Toggle for F2837xD CPU1.
//
//! \addtogroup dual_example_list
//! <h1> IPC GPIO toggle </h1>
//!
//! This example shows GPIO input on the local CPU triggering an output on the
//! remote CPU. A GPIO input change on CPU01 causes an output change on CPU02
//! and vice versa. \n
//! CPU1 has control of GPIO31 , GPIO15 and GPIO14.\n
//! CPU2 has control of GPIO34 , GPIO10 and GPIO11.\n
//!
//! The IPC is used to signal a change on the CPU's input pin.\n
//!
//! \b Hardware \b Connections
//!   - connect GPIO15 to GPIO11
//!   - connect GPIO14 to GPIO10
//!
//! \b Watch \b Pins
//!   - GPIO34 - output on CPU2
//!   - GPIO11 - input on CPU2
//!   - GPIO31 - output on CPU1
//!   - GPIO14 - input on CPU1
//!   - GPIO10 - square wave output on CPU02
//!   - GPIO15 - square wave output on CPU01
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.12.00.00 $
// $Release Date: Fri Feb 12 19:03:23 IST 2021 $
// $Copyright:
// Copyright (C) 2013-2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//

#include "driverlib.h"
#include "ipc.h"
#include "device.h"
#include "inc/hw_ipc.h"
#include "ECPUTimer.h"

#pragma DATA_SECTION(readData, "PUTBUFFER");
uint16_t readData[10];

static __interrupt void cpuTimer0ISR(void);
static __interrupt void cpuTimer1ISR(void);

//
// Main
//
uint32_t CPU1mainCNT;
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();


#ifdef _STANDALONE
#ifdef _FLASH
    //
    // Send boot command to allow the CPU2 application to begin execution
    //
    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
#else
    //
    // Send boot command to allow the CPU2 application to begin execution
    //
    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
#endif
#endif

    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
    Device_initGPIO();

    GPIO_setPadConfig(31, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO31
    GPIO_writePin(31, 0);                          // Load output latch
    GPIO_setPinConfig(GPIO_31_GPIO31);             // GPIO31 = GPIO31
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);  // GPIO31 = output

    //
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();


    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    Interrupt_register(INT_TIMER1, &cpuTimer1ISR);
    initCPUTimers(CPUTIMER0_BASE);
    initCPUTimers(CPUTIMER1_BASE);
    configCPUTimer(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ, 100000.0F);  //100ms
    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, 100000.0F);  //10ms



    IPC_clearFlagLtoR(IPC_CPU1_L_CPU2_R, IPC_FLAG_ALL);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    IPC_sync(IPC_CPU1_L_CPU2_R, IPC_FLAG31);

    Interrupt_enable(INT_TIMER0);
    Interrupt_enable(INT_TIMER1);
    CPUTimer_startTimer(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER1_BASE);

    EINT;
    ERTM;



    while(1)
    {
        CPU1mainCNT++;
    }

}
uint32_t tiCNT1;
uint32_t tiCNT2;
uint32_t Sdata = 0xDEAD;
uint32_t respdata = 0;
static __interrupt void
cpuTimer0ISR(void)
{
    tiCNT1++;
    IPC_sendCommand(IPC_CPU1_L_CPU2_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                    0x01, (uint32_t)readData, Sdata);

    IPC_waitForAck(IPC_CPU1_L_CPU2_R, IPC_FLAG0);

    respdata = IPC_getResponse(IPC_CPU1_L_CPU2_R);
    if(respdata == 0xAAAA)//pass
    {
        GPIO_togglePin(31);
    }
    else
    {

    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

static __interrupt void
cpuTimer1ISR(void)
{
    tiCNT2++;

}
//
// End of file
//
