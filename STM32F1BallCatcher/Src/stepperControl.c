/*
 * stepperControl.c
 *
 *  Created on: May 9, 2018
 *      Author: saogalde
 */
#include "stepperControl.h"
#include "main.h"

void writeStepperPin(stepper_pin pin, uint8_t state){
	HAL_GPIO_WritePin(pin.PIN_PORT, pin.PIN_NO, state);
}

uint32_t saturate(int64_t speed, stepper* st){
	if(speed>=st->maxPeriodCounts) return st->maxPeriodCounts;
	else if(speed<=st->minPeriodCounts) return st->minPeriodCounts;
	else return speed;
}

void changeMicrostepping(stepper* st, stepper_microstep ms){
	/*writeStepperPin(st->STPIN_M0,(1&ms));
	writeStepperPin(st->STPIN_M1,(2&ms)>>1);
	writeStepperPin(st->STPIN_M2,(4&ms)>>2);*/
	st->microstepping = ms;
	/*HAL_GPIO_WritePin(st->STPIN_M0.PIN_PORT, st->STPIN_M0.PIN_NO, (1&ms));
	HAL_GPIO_WritePin(st->STPIN_M1.PIN_PORT, st->STPIN_M1.PIN_NO, ;
	HAL_GPIO_WritePin(st->STPIN_M2.PIN_PORT, st->STPIN_M2.PIN_NO, (4&ms)>>2);*/
}

/*void Stepper_InitializeHardware(stepper* st, TIM_HandleTypeDef* htim, uint8_t chtim,\
		GPIO_TypeDef* M0Port,uint16_t M0, \
		GPIO_TypeDef* M1Port,uint16_t M1, \
		GPIO_TypeDef* M2Port,uint16_t M2, \
		GPIO_TypeDef* DIRPort,uint16_t DIR, \
		GPIO_TypeDef* ENPort,uint16_t EN){*/

void Stepper_InitializeHardware(stepper* st, TIM_HandleTypeDef* htim, uint8_t chtim,\
		GPIO_TypeDef* DIRPort,uint16_t DIR, \
		GPIO_TypeDef* ENPort,uint16_t EN){

	st->STEP_TIMER 			= htim;
	st->STEP_CHANNEL			= chtim;
	/*st->STPIN_M0.PIN_PORT 	= M0Port;
	st->STPIN_M0.PIN_NO 		= M0;
	st->STPIN_M1.PIN_PORT 	= M1Port;
	st->STPIN_M1.PIN_NO 		= M1;
	st->STPIN_M2.PIN_PORT 	= M2Port;
	st->STPIN_M2.PIN_NO 		= M2;*/
	st->STPIN_DIR.PIN_PORT 	= DIRPort;
	st->STPIN_DIR.PIN_NO 	= DIR;
	st->STPIN_EN.PIN_PORT 	= ENPort;
	st->STPIN_EN.PIN_NO 		= EN;
}

void Stepper_SetDefaultConfig(stepper* st, stepper_microstep ms){
	//changeMicrostepping(st, ms);
	//Stepper_Enable(st);
	/*if(st->status & SS_STOPPED){
		st->status = SS_STARTING;
	}*/
	/*if(st->status & SS_UNDEFINED){
		st->status = SS_STOPPED;
	}*/
	//st->maxStepsSec = MOTOR_MAX_PERIOD_COUNTS;
	//st->minStepsSec = MOTOR_MIN_PERIOD_COUNTS;
	st->status = SS_STOPPED;
}

void Stepper_Enable(stepper* st){
	writeStepperPin(st->STPIN_EN,SET);
}

void Stepper_Disable(stepper* st){
	writeStepperPin(st->STPIN_EN,RESET);
}

void Stepper_ChangeDir(stepper* st, FlagStatus status){
	writeStepperPin(st->STPIN_DIR, status);
	st->dir = (uint8_t)status;
}

void Stepper_Controller(stepper* st){
	stepper_status status = st->status;
	if (status & SS_STOPPED) {
		if (st->targetPosition != st->currentPosition) {
			st->status = SS_STARTING;
			Stepper_Enable(st);
			modprintf("PWM_1\n");
			HAL_TIM_PWM_Start(st->STEP_TIMER, st->STEP_CHANNEL);
			/*st->currentStepsSec = st->minStepsSec;
			st->initialPosition = st->currentPosition;
			st->accelPrescaler = 2;*/
		}
	}
	//Stepper_SetTimerByStepsSec(st);
	// check if we need to brake.
	// SIMPLEST CRITERIA BY NOW :( = start braking when reached halfway
	/*if(!(status & SS_BRAKING)){ // if not braking
		if (abs(st->currentPosition - st->targetPosition) <= abs(st->currentPosition - st->targetPosition)/2){
			st->status &= ~SS_BRAKECORRECTION;
			st->status |= SS_BRAKING;
		}

	}*/
	/*st->accelPrescaler--;
	if(st->accelPrescaler <= 0){
		st->accelPrescaler = 2;
	}*/


}

void Stepper_PulseUpdate(stepper* st){
	//switch (st->status & ~(SS_BRAKING|SS_BRAKECORRECTION)){ // masking so we dont loose info about direction
	switch (st->status){
	case SS_STARTING:
		if (st->currentPosition > st->targetPosition){
			st->status = SS_RUNNING_BACKWARD;
			Stepper_ChangeDir(st,RESET);
			//writeStepperPin(st->STPIN_DIR, RESET);
		} else if (st->currentPosition < st->targetPosition){
			st->status = SS_RUNNING_FORWARD;
			Stepper_ChangeDir(st,SET);
			//writeStepperPin(st->STPIN_DIR, SET);
		} else if (st->currentPosition == st->targetPosition) {
			st->status = SS_STOPPED;
			modprintf("PWM_0\n");
			HAL_TIM_PWM_Stop(st->STEP_TIMER, st->STEP_CHANNEL);
		}
		break;
	case SS_RUNNING_FORWARD:
    case SS_RUNNING_BACKWARD:
    	//modprintf("SS_RUNNING_FORWARD\n");
    	//if(st->calib) st->currentPosition += GetStepDirectionUnit(st);
    	//st->currentPosition += GetStepDirectionUnit(st);
    	//else{
    	st->minStepsSec = (uint32_t)(st->proportionalTerm*(Stepper_RemainingSteps(st))*(st->speedSpan));
    	st->currentStepsSec = saturate((int64_t)(st->maxPeriodCounts - st->proportionalTerm*(Stepper_RemainingSteps(st))*(st->speedSpan)*0.01F), st);
    	__HAL_TIM_SET_AUTORELOAD(st->STEP_TIMER,st->currentStepsSec);
    	//}
    	//modprintf("STP: %d\n", Stepper_RemainingSteps(st));
    	if (st->currentPosition > st->targetPosition){
			st->status = SS_RUNNING_BACKWARD;
			//writeStepperPin(st->STPIN_DIR, RESET);
			Stepper_ChangeDir(st,RESET);
		} else if (st->currentPosition < st->targetPosition){
			st->status = SS_RUNNING_FORWARD;
			//writeStepperPin(st->STPIN_DIR, SET);
			Stepper_ChangeDir(st,SET);
		} else if (st->currentPosition == st->targetPosition) {
			st->status = SS_STOPPED;
			HAL_TIM_PWM_Stop(st->STEP_TIMER, st->STEP_CHANNEL);
			modprintf("PWM_0\n");
			//Stepper_Disable(st);
		}
      // The actual pulse has been generated by previous timer run.
    	/*if(st->currentPosition >= st->maxPosition-30){
    		int i = 0;
    		i++;
    	}*/
    	break;
	}
}

void Stepper_TurnOnTimer_IT(stepper* st){
	__HAL_TIM_ENABLE_IT(st->STEP_TIMER, TIM_IT_UPDATE);
}
void Stepper_TurnOffTimer_IT(stepper* st){
	__HAL_TIM_DISABLE_IT(st->STEP_TIMER, TIM_IT_UPDATE);
}

int32_t GetStepDirectionUnit(stepper* st){
	    return (st->status & SS_RUNNING_BACKWARD) ? -1 : 1;
	}
int32_t Stepper_isItBraking(stepper* st){
	return (st->status & SS_BRAKING) ? -1 : 1;
}

int32_t Stepper_RemainingSteps(stepper* st){
	return (st->targetPosition - st->currentPosition);
}
/*

void Stepper_SetTimerByStepsSec(stepper* st){
	//uint32_t timerTicks = STEP_TIMER_CLOCK / step -> currentSPS;
	int32_t sign = Stepper_isItBraking(st);
	st->currentStepsSec += st->accelStepsSec*sign;
	st->STEP_TIMER->Instance->ARR = st->currentStepsSec;
}
*/

bool Stepper_AtTarget(stepper* st){
	if(st->currentPosition == st->targetPosition) return true;
	else return false;
}
