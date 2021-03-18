/*
 * Timer.h
 *
 *  Created on: 2020. 4. 27.
 *      Author: msi
 */

#ifndef HEADER_CPUTIMER_H_
#define HEADER_CPUTIMER_H_

#include "driverlib.h"
#include "device.h"
/**
@brief      CPU타이머 초기화 수행
@param[in]  uint32_t cpuTimer
@param[out] void.
@param[in,out] void.
@return     void.
void Run_Button(void);
 */
extern void initCPUTimers(uint32_t cpuTimer);
/**
@brief      CPU타이머 설정 수행
@param[in]  uint32_t cpuTimer, float freq, float period
@param[out] void.
@param[in,out] void.
@return     void.
void Run_Button(void);
 */
extern void configCPUTimer(uint32_t cpuTimer, float freq, float period);
#endif /* HEADER_CPUTIMER_H_ */
