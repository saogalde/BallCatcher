/*

 * stepperControl.h
 *
 *  Created on: May 9, 2018
 *      Author: saogalde
 *      https://github.com/omuzychko/StepperHub
 */

#ifndef STEPPERCONTROL_H_
#define STEPPERCONTROL_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef enum{
	MICROSTEP_1 		= 0x00,
	MICROSTEP_2 		= 0x01,
	MICROSTEP_4 		= 0x02,
	MICROSTEP_8 		= 0x03,
	MICROSTEP_16		= 0x04,
	MICROSTEP_32		= 0x05,
} stepper_microstep;

typedef enum {
    SS_UNDEFINED         = 0x00,
    SS_RUNNING_BACKWARD  = 0x01,
    SS_RUNNING_FORWARD   = 0x02,
    SS_STARTING          = 0x04,
    SS_BRAKING          = 0x10,
    SS_BRAKECORRECTION   = 0x20,
    SS_STOPPED           = 0x80
} stepper_status;

/*
typedef enum {
    SERR_OK                     = 0,
    SERR_NOMORESTATESAVAILABLE  = 1,
    SERR_MUSTBESTOPPED          = 2,
    SERR_STATENOTFOUND          = 3,
    SERR_LIMIT                  = 4
} stepper_error;*/

typedef struct{
	GPIO_TypeDef* PIN_PORT;
	uint16_t PIN_NO;
} stepper_pin;

typedef struct{
	stepper_status status;
	stepper_microstep microstepping;
	bool calib;

	// positions
	volatile int32_t initialPosition;
	volatile int32_t currentPosition;
	volatile int32_t targetPosition;
	uint32_t maxPosition;

	// speeds
	volatile int32_t minStepsSec;
	volatile int32_t maxStepsSec;
	volatile int32_t currentStepsSec;
	uint32_t speedSpan;

	// accelerations
	volatile int32_t accelStepsSec;
	volatile int32_t accelPrescaler;
	volatile int32_t proportionalTerm;
	//volatile int32_t accelStepsSec;

	// associated hardware
	TIM_HandleTypeDef* STEP_TIMER;
	uint16_t STEP_CHANNEL;
	stepper_pin STPIN_M0;
	stepper_pin STPIN_M1;
	stepper_pin STPIN_M2;
	stepper_pin STPIN_EN;
	stepper_pin STPIN_DIR;
} stepper;

void writeStepperPin(stepper_pin pin, uint8_t state);
void Stepper_ChangeMicrostepping(stepper* st,stepper_microstep);
void Stepper_InitializeHardware(stepper* st, TIM_HandleTypeDef* htim, uint8_t chtim,\
		GPIO_TypeDef* M0Port,uint16_t M0, \
		GPIO_TypeDef* M1Port,uint16_t M1, \
		GPIO_TypeDef* M2Port,uint16_t M2, \
		GPIO_TypeDef* DIRPort,uint16_t DIR, \
		GPIO_TypeDef* ENPort,uint16_t EN);

int32_t GetStepDirectionUnit(stepper* st);
void Stepper_SetDefaultConfig(stepper* st, stepper_microstep ms);
void Stepper_Enable(stepper* st);
void Stepper_Disable(stepper* st);
void Stepper_TurnOnTimer_IT(stepper* st);
void Stepper_TurnOffTimer_IT(stepper* st);
void Stepper_Controller(stepper* st);
void Stepper_PulseUpdate(stepper* st);
bool Stepper_AtTarget(stepper* st);
int32_t Stepper_RemainingSteps(stepper* st);
void watcher(stepper* st1,stepper* st2);
#endif /* STEPPERCONTROL_H_ */
