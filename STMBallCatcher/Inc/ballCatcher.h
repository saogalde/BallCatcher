/*
 * ballCatcher.h
 *
 *  Created on: May 9, 2018
 *      Author: saogalde
 */

#ifndef BALLCATCHER_H_
#define BALLCATCHER_H_

#include "stepperControl.h"
#include <stdbool.h>

#define DEFAULT_MICROSTEP		MICROSTEP_16
#define STEPS_BY_TURN			(float)(200*16)
#define PULLEY_RADIUS			2.5		// in cm
#define PULLEY_PERIMETER		(float)(2.0*3.14f*PULLEY_RADIUS)
#define CAPTURE_SIZE_CM_X		59.0F		// in cm
#define CAPTURE_SIZE_CM_Y		42.0F		// in cm
#define CORRECTION_X			1.3F
#define CORRECTION_Y			0.84F
#define CAPTURE_SIZE_STEPS_X	(int)((CAPTURE_SIZE_CM_X/PULLEY_PERIMETER)*STEPS_BY_TURN*CORRECTION_X)	// in 1*steps
#define CAPTURE_SIZE_STEPS_Y	(int)((CAPTURE_SIZE_CM_Y/PULLEY_PERIMETER)*STEPS_BY_TURN*CORRECTION_Y) 	// in 1*steps

typedef enum{
	MODE_CALIB 			= 0x01,
	MODE_WAITING		= 0x02,
	MODE_CAPTURE		= 0x04,
	MODE_POSTCAPTURE	= 0x08
} ballcatcher_mode;

typedef struct{
	uint8_t x;
	uint8_t y;
} requested_position;

requested_position req_pos;
requested_position req_cat;
ballcatcher_mode mode;

stepper motorEjeCentral;
stepper motorEjeBasal;
TIM_HandleTypeDef* ballCatcherTimer;
TIM_HandleTypeDef* stepperUpdateTimer;
TIM_HandleTypeDef* servoTimer;
I2C_HandleTypeDef* hi2c;
//UART_HandleTypeDef* uartDevice;
SPI_HandleTypeDef* spiDevice;
DMA_HandleTypeDef* hdma;
volatile bool captured;
volatile bool ballDetected;
uint8_t rxBuffer[4];
volatile bool receiving_pos;
volatile bool receiving_cat;
volatile bool x_otherwise_y;
// initializes full system
void initBallCatcher(TIM_HandleTypeDef* htimBasal, TIM_HandleTypeDef* htimCentral, \
		TIM_HandleTypeDef* stUp, TIM_HandleTypeDef* bcTim, TIM_HandleTypeDef* servoTim, \
		I2C_HandleTypeDef* i2cdevice, SPI_HandleTypeDef* ud,
		DMA_HandleTypeDef* dmad);
void BallCatcher_Controller(stepper* st1, stepper* st2);
void BallCatcher_TurnOnTimerIT();
void BallCatcher_TurnOffTimerIT();
void BallCatcher_TurnOnSerialIT();
void BallCatcher_TurnOffSerialIT();
void BallCatcher_SetTargetPosition(requested_position pos);
void BallCatcher_TurnOffMotorsIT();
void BallCatcher_TurnOnMotorsIT();
void BallCatcher_OpenDoor();
void BallCatcher_CloseDoor();

// calibrates the system by going home and setting the 0,0 coordinate
void calibrate();

// serial functionality
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
#endif /* BALLCATCHER_H_ */
