//###########################################################################
//
// FILE:   ipc_gpio_toggle_cpu02.c
//
// TITLE:  IPC GPIO Toggle for F2837xD CPU2.
//
// This example shows GPIO input on the local CPU triggering an output on the
// remote CPU. A GPIO input change on CPU01 causes an output change on CPU02
// and vice versa.
// CPU1 has control of GPIO31 , GPIO15 and GPIO14.
// CPU2 has control of GPIO34 , GPIO10 and GPIO11.
//
// The IPC is used to signal a change on the CPU's input pin.
//
// \b Hardware \b Connections
//   - connect GPIO15 to GPIO11
//   - connect GPIO14 to GPIO10
//
// \b Watch \b Pins
//   - GPIO34 - output on CPU2
//   - GPIO11 - input on CPU2
//   - GPIO31 - output on CPU1
//   - GPIO14 - input on CPU1
//   - GPIO10 - square wave output on CPU02
//   - GPIO15 - square wave output on CPU01
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
#include "device.h"
#include "ipc.h"
#include "inc/hw_ipc.h"

static __interrupt void IPC_ISR0(void);
//
// Main
//
uint32_t CPU2mainCNT;
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
    // This is configured by CPU1

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    IPC_clearFlagLtoR(IPC_CPU2_L_CPU1_R, IPC_FLAG_ALL);
    IPC_registerInterrupt(IPC_CPU2_L_CPU1_R, IPC_INT0, IPC_ISR0);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    IPC_sync(IPC_CPU2_L_CPU1_R, IPC_FLAG31);

    while(1)
    {
        CPU2mainCNT++;
    }
}
uint32_t cmd, addr, data;
uint16_t status = 0;
uint32_t IPCintCNT;
static __interrupt void IPC_ISR0(void)
{
    IPCintCNT++;
    IPC_readCommand(IPC_CPU2_L_CPU1_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                    &cmd, &addr, &data);

    if(cmd == 0x01)
    {
        status = 1;

        if(data != 0xDEAD)
        {
            status = 0;
        }
    }

    if(status == 1)
    {
        IPC_sendResponse(IPC_CPU2_L_CPU1_R, 0xAAAA);
    }
    else
    {
        IPC_sendResponse(IPC_CPU2_L_CPU1_R, 0x5555);
    }
    //
    // Acknowledge the flag
    //
    IPC_ackFlagRtoL(IPC_CPU2_L_CPU1_R, IPC_FLAG0);

    //
    // Acknowledge the PIE interrupt.
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
//
// End of file
//
