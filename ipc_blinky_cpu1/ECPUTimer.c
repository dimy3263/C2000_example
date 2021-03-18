/*
 * Timer.c
 *
 *  Created on: 2020. 4. 27.
 *      Author: msi
 */

#include <ECPUTimer.h>

// initCPUTimers - This function initializes all three CPU timers
// to a known state.
//
void
initCPUTimers(uint32_t cpuTimer)
{
    //
    // Initialize timer period to maximum
    //
    CPUTimer_setPeriod(cpuTimer, 0xFFFFFFFFU);

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(cpuTimer, 0);


    //
    // Make sure timer is stopped
    //
    CPUTimer_stopTimer(cpuTimer);


    //
    // Reload all counter register with period value
    //
    CPUTimer_reloadTimerCounter(cpuTimer);


}


//
// configCPUTimer - This function initializes the selected timer to the
// period specified by the "freq" and "period" parameters. The "freq" is
// entered as Hz and the period in uSeconds. The timer is held in the stopped
// state after configuration.
//
void
configCPUTimer(uint32_t cpuTimer, float freq, float period)
{
    float temp;

    //
    // Initialize timer period:
    //
    temp = (freq / 1000000.0f) * period;
    CPUTimer_setPeriod(cpuTimer, (uint32_t)temp);

    //
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CPUTimer_setPreScaler(cpuTimer, 0);

    //
    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    //
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                              CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_enableInterrupt(cpuTimer);
}
