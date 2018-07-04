/*
 * ballCatcher.c
 *
 *  Created on: May 9, 2018
 *      Author: saogalde
 */

#include "ballCatcher.h"
#include "stepperControl.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "lcd_i2c.h"
#include "main.h"

volatile uint8_t superbuffer[256];
volatile uint8_t i = 0;
volatile uint8_t uartIndicador = 0;

void initBallCatcher(TIM_HandleTypeDef* htimBasal, TIM_HandleTypeDef* htimCentral, \
		TIM_HandleTypeDef* stUp, TIM_HandleTypeDef* bcTim, \
		I2C_HandleTypeDef* i2cdevice, UART_HandleTypeDef* ud,
		DMA_HandleTypeDef* dmad){

	// variable init
	ballCatcherTimer = bcTim;
	stepperUpdateTimer = stUp;
	//servoTimer = servoTim;
	uartDevice = ud;
	hi2c = i2cdevice;
	hdma = dmad;
	captured = false;
	ballDetected = false;
	receiving_pos = false;
	x_otherwise_y = true;

	// screen init
	LCDInit(i2cdevice);
	LCD_SendLines("BIENVENIDO A", "BALLCATCHER");
#ifndef MOTOR_DEBUG
	HAL_Delay(1000);
#endif
	// motor zero initialization
	memset(&motorEjeCentral, 0, sizeof(stepper));
	memset(&motorEjeBasal, 0, sizeof(stepper));
	//HAL_TIM_PWM_Start(servoTimer, TIM_CHANNEL_1);
	//BallCatcher_CloseDoor();

	// motor general initialization
	Stepper_InitializeHardware(&motorEjeBasal, htimBasal, STEP_CHANNEL_BASAL, M0_1_GPIO_Port, M0_1_Pin, \
			M1_1_GPIO_Port, M1_1_Pin, M2_1_GPIO_Port, M2_1_Pin,  DIR_1_GPIO_Port, DIR_1_Pin, MOTOR_EN_1_GPIO_Port, MOTOR_EN_1_Pin);
	Stepper_InitializeHardware(&motorEjeCentral, htimCentral, STEP_CHANNEL_CENTRAL, M0_1_GPIO_Port, M0_1_Pin, \
				M1_2_GPIO_Port, M1_2_Pin, M2_2_GPIO_Port, M2_2_Pin, DIR_2_GPIO_Port, DIR_2_Pin, MOTOR_EN_2_GPIO_Port, MOTOR_EN_2_Pin);
	//Stepper_SetDefaultConfig(&motorEjeCentral, DEFAULT_MICROSTEP);
	//Stepper_SetDefaultConfig(&motorEjeBasal, DEFAULT_MICROSTEP);
	Stepper_SetDefaultConfig(&motorEjeCentral, MICROSTEP_16);
	Stepper_SetDefaultConfig(&motorEjeBasal, MICROSTEP_16);

	// CONFIGURACION DE LIMITES PARA AMBOS MOTORES
	motorEjeBasal.maxPosition = CAPTURE_SIZE_STEPS_X;
	motorEjeCentral.maxPosition = CAPTURE_SIZE_STEPS_Y;
	motorEjeBasal.maxPeriodCounts = MOTOR_MAX_PERIOD_COUNTS;
	motorEjeBasal.minPeriodCounts = MOTOR_MIN_PERIOD_COUNTS_X;
	motorEjeCentral.maxPeriodCounts = MOTOR_MAX_PERIOD_COUNTS;
	motorEjeCentral.minPeriodCounts = MOTOR_MIN_PERIOD_COUNTS_Y;
	modprintf("MAX X STEPS: %d, MAX Y STEPS: %d\n", CAPTURE_SIZE_STEPS_X, CAPTURE_SIZE_STEPS_Y);
	motorEjeBasal.speedSpan = (MOTOR_MAX_PERIOD_COUNTS-MOTOR_MIN_PERIOD_COUNTS_X);
	motorEjeCentral.speedSpan = (MOTOR_MAX_PERIOD_COUNTS-MOTOR_MIN_PERIOD_COUNTS_Y);

	modprintf("SPEEDSPAN X STEPS: %d, SPEEDSPAN Y STEPS: %d\n", motorEjeBasal.speedSpan, motorEjeCentral.speedSpan);
	modprintf("CUTSPEED X: %d, Y: %d\n", MOTOR_CUTSPEED_X, MOTOR_CUTSPEED_Y);
	// CONFIGURACION DE PARAMETROS PARA EL CONTROL
	motorEjeBasal.Kp = MOTOR_CUTSPEED_X/CAPTURE_SIZE_STEPS_X;
	motorEjeCentral.Kp = 0;
	motorEjeBasal.Ki = MOTOR_KI_X;
	motorEjeCentral.Ki = MOTOR_KI_Y;
	motorEjeBasal.Kd = 0;
	motorEjeCentral.Kd = 0;
	motorEjeBasal.integratorUpperLimit = motorEjeBasal.speedSpan;
	motorEjeCentral.integratorUpperLimit = motorEjeCentral.speedSpan;
	motorEjeBasal.integratorLowerLimit = -motorEjeBasal.speedSpan;
	motorEjeCentral.integratorLowerLimit = -motorEjeCentral.speedSpan;
	modprintf("CONTROL X P/I/D: %d/%d/%d, CONTROL Y P/I/D: %d/%d/%d\n", motorEjeBasal.Kp, motorEjeBasal.Ki, motorEjeBasal.Kd, motorEjeCentral.Kp, motorEjeCentral.Ki, motorEjeCentral.Kd);

	// serial init
	__HAL_UART_FLUSH_DRREGISTER(ud);
	HAL_UART_Receive_DMA(ud, &rxBuffer, 1);
	HAL_UART_DMAPause(ud);
	motorEjeBasal.targetPosition = 0;
	motorEjeCentral.targetPosition = 0;

	// ball catcher general initialization
	mode = MODE_CALIB;
	BallCatcher_TurnOnTimerIT();

}

void BallCatcher_Controller(stepper* st1, stepper* st2){
	switch(mode){
	case MODE_CALIB:
		HAL_UART_DMAPause(uartDevice);
		BallCatcher_TurnOffTimerIT();
		#ifndef MOTOR_DEBUG
		calibrate(); // this function already turn the motors on :)
		#endif
		#ifdef MOTOR_DEBUG
		motor_debug();
		#endif
		mode = MODE_WAITING;
		BallCatcher_TurnOnTimerIT();
		HAL_UART_DMAResume(uartDevice);
		break;
	case MODE_WAITING:
		if(ballDetected){
			//prender TIM11
			//activar interrupcion de TIM11
			LCD_SendLines("PELOTA     **", "DETECTADA! **");
			mode = MODE_CAPTURE;
		}
		break;
	case MODE_CAPTURE:
		//modprintf("capture\n");
		break;
	case MODE_POSTCAPTURE:
		//if(captured)
		break;

	}
}

void calibrate(){
	BallCatcher_TurnOffMotorsIT();
	/*motorEjeCentral.calib = true;
	#ifndef BASE_MOTOR_ONLY
	motorEjeBasal.calib = true;
	#endif*/
	// write to screen "CALIBRANDO \n PONER MOTORES EN HOME"
	LCD_SendLines("CALIBR: PONER", "MOTORES EN HOME!");
	HAL_Delay(1200);
	LCD_SendLines("PRESIONAR BOTON", "PARA CONFIRMAR");
	//while(HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin));
	motorEjeCentral.currentPosition = 0;
	motorEjeBasal.currentPosition = 0;
	LCD_SendLines("CALIBRADO!", "");
	//HAL_Delay(500);

	LCD_SendLines("CALIBR.RASPBERRY", "MOT AL CENTRO...");
	//motorEjeCentral.targetPosition = (uint32_t)(motorEjeCentral.maxPosition/2);
	//motorEjeBasal.targetPosition = (uint32_t)(motorEjeBasal.maxPosition/2);
	//motorEjeCentral.targetPosition = 0;
	//motorEjeBasal.targetPosition = 0;


	//motorEjeCentral.targetPosition = (int)(motorEjeCentral.maxPosition);
	//motorEjeBasal.targetPosition = (int)(motorEjeBasal.maxPosition);
	//motorEjeCentral.targetPosition = 2;
	//motorEjeBasal.targetPosition = 2;
	BallCatcher_TurnOnMotorsIT();

	// wait until center is reached
	#ifndef BASE_MOTOR_ONLY
	while(!(Stepper_AtTarget(&motorEjeCentral)) || !(Stepper_AtTarget(&motorEjeBasal)));
	#else
	while(!(Stepper_AtTarget(&motorEjeBasal)));
	#endif

	/*
	motorEjeCentral.calib = false;
	#ifndef BASE_MOTOR_ONLY
	motorEjeBasal.calib = false;
	#endif*/
	LCD_SendLines("CALIBRACION", "COMPLETA!");
	//HAL_Delay(1000);
	LCD_SendLines("LISTO PARA", "CAPTURAR!");
	//HAL_Delay(1000);
	LCD_SendLines("DAME TU", "MEJOR TIRO ;)");
}

void motor_debug(){
	BallCatcher_TurnOffMotorsIT();
	motorEjeCentral.calib = true;
	#ifndef BASE_MOTOR_ONLY
	motorEjeBasal.calib = true;
	#endif
	// write to screen "CALIBRANDO \n PONER MOTORES EN HOME"
	LCD_SendLines("MODO DEBUG", "DE MOTORES...");
	motorEjeCentral.currentPosition = 0;
	motorEjeBasal.currentPosition = 0;

	BallCatcher_TurnOnMotorsIT();


	// wait until center is reached
	motorEjeCentral.targetPosition = (uint32_t)(motorEjeCentral.maxPosition);
	motorEjeBasal.targetPosition = 0;
	// ESQUINA 1
	#ifndef BASE_MOTOR_ONLY
	while(!(Stepper_AtTarget(&motorEjeCentral)) || !(Stepper_AtTarget(&motorEjeBasal)));
	#else
	while(!(Stepper_AtTarget(&motorEjeBasal)));
	#endif

	// wait until center is reached
	motorEjeCentral.targetPosition = (uint32_t)(motorEjeCentral.maxPosition);
	motorEjeBasal.targetPosition = (uint32_t)(motorEjeBasal.maxPosition);
	#ifndef BASE_MOTOR_ONLY
	while(!(Stepper_AtTarget(&motorEjeCentral)) || !(Stepper_AtTarget(&motorEjeBasal)));
	#else
	while(!(Stepper_AtTarget(&motorEjeBasal)));
	#endif

	// wait until center is reached
	motorEjeCentral.targetPosition = 0;
	motorEjeBasal.targetPosition = (uint32_t)(motorEjeBasal.maxPosition);
	#ifndef BASE_MOTOR_ONLY
	while(!(Stepper_AtTarget(&motorEjeCentral)) || !(Stepper_AtTarget(&motorEjeBasal)));
	#else
	while(!(Stepper_AtTarget(&motorEjeBasal)));
	#endif

	// wait until center is reached
	motorEjeCentral.targetPosition = 0;
	motorEjeBasal.targetPosition = 0;
	#ifndef BASE_MOTOR_ONLY
	while(!(Stepper_AtTarget(&motorEjeCentral)) || !(Stepper_AtTarget(&motorEjeBasal)));
	#else
	while(!(Stepper_AtTarget(&motorEjeBasal)));
	#endif


	motorEjeCentral.calib = false;
	#ifndef BASE_MOTOR_ONLY
	motorEjeBasal.calib = false;
	#endif
	LCD_SendLines("CALIBRACION", "COMPLETA!");
}


void BallCatcher_TurnOffMotorsIT(){
	HAL_TIM_Base_Stop_IT(stepperUpdateTimer);
	Stepper_TurnOffTimer_IT(&motorEjeBasal);
	Stepper_TurnOffTimer_IT(&motorEjeCentral);
}

void BallCatcher_TurnOnMotorsIT(){
	Stepper_TurnOnTimer_IT(&motorEjeBasal);
	Stepper_TurnOnTimer_IT(&motorEjeCentral);
	HAL_TIM_Base_Start_IT(stepperUpdateTimer);
}

void BallCatcher_TurnOnTimerIT(){
	HAL_TIM_Base_Start_IT(ballCatcherTimer);
}
void BallCatcher_TurnOffTimerIT(){
	HAL_TIM_Base_Stop_IT(ballCatcherTimer);
}

void BallCatcher_TurnOnSerialIT(){

}
void BallCatcher_TurnOffSerialIT(){

}

void BallCatcher_SetTargetPosition(requested_position pos){
	motorEjeBasal.targetPosition = (uint32_t)(motorEjeBasal.maxPosition*( (float)(pos.x/254.0f) ));
	motorEjeCentral.targetPosition = (uint32_t)(motorEjeCentral.maxPosition*( (float)(pos.y/254.0f) ));
}

void BallCatcher_SetCurrentPosition(requested_position pos){
	motorEjeBasal.currentPosition = (uint32_t)(motorEjeBasal.maxPosition*( (float)(pos.x/254.0f) ));
	motorEjeCentral.currentPosition = (uint32_t)(motorEjeCentral.maxPosition*( (float)(pos.y/254.0f) ));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	__HAL_UART_FLUSH_DRREGISTER(huart); // Clear the buffer to prevent overrun

	if(rxBuffer == 255){
		ballDetected = true;
		receiving_pos = true;
		receiving_cat = false;
		x_otherwise_y = true;
	}
	else{
		if(receiving_pos){
			if(x_otherwise_y){
				req_pos.x = rxBuffer;
				superbuffer[i++]=rxBuffer;
				x_otherwise_y = false;
			}
			else{
				req_pos.y = rxBuffer;
				superbuffer[i++]=rxBuffer;
				x_otherwise_y = true;
				receiving_pos = false;
				receiving_cat = true;
			}
		}
		else if(receiving_cat){
			if(x_otherwise_y){
				req_cat.x = rxBuffer;
				superbuffer[i++]=rxBuffer;
				x_otherwise_y = false;
			}
			else{
				req_cat.y = rxBuffer;
				superbuffer[i++]=rxBuffer;
				x_otherwise_y = true;
				receiving_pos = false;
				receiving_cat = false;
				BallCatcher_SetTargetPosition(req_pos);
				BallCatcher_SetCurrentPosition(req_cat);
				//modprintf("%d %d %d %d\n", req_pos.x, req_pos.y, req_cat.x, req_cat.y);

			}

		}
	}

	/*req_cat.x = rxBuffer[3];
	req_cat.y = rxBuffer[2];
	req_cat.x = rxBuffer[1];
	req_cat.y = rxBuffer[0];
	modprintf("%d %d %d %d\n", rxBuffer[0], rxBuffer[1], rxBuffer[2], rxBuffer[3]);
	HAL_SPI_Receive_IT(spiDevice, rxBuffer, 4);
	HAL_UART_Transmit(uartDevice, (char)rxBuffer, 1, 5);*/
	HAL_UART_Receive_DMA(uartDevice, &rxBuffer, 1);
	//uartIndicador ^= 0x01;
	uartIndicador++;
}

void BallCatcher_OpenDoor(){
	servoTimer->Instance->CCR1 = SERVO_OPEN;
}
void BallCatcher_CloseDoor(){
	servoTimer->Instance->CCR1 = SERVO_CLOSED;
}


