/*
 * Accel_stepper.h
 *
 *  Created on: Mar 26, 2022
 *      Author: Phanny
 *      This Libray is base on AVR446: Linear speed control of stepper motor
 *      For document you can find at
 *      https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf
 */

#ifndef INC_ACCEL_STEPPER_H_
#define INC_ACCEL_STEPPER_H_

#include "stdlib.h"
#include "math.h"
#include <stdint.h>
#include "gpio.h"
#include "tim.h"

/*
 * Increase scale for faster calculate
 * SPR : step per round
 * TIM_FREQ : frequency of timer interrupt
 *
 */

#define SPR 6400 //1/32
#define TIM_FREQ 1000000
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define AxTIM_FREQ ((long)(ALPHA*TIM_FREQ))     // (ALPHA / TIM_FREQ)
#define TIM_FREQ_SCALE ((int)((TIM_FREQ*0.676))) //scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)              //2*@ : ALPHA*20000 must divide by 10000 after use

typedef enum{
	STOP,
	ACCEL,
	DECEL,
	RUN
}ramp_state_t;
typedef enum{
	STEPPER1,
	STEPPER2,
	STEPPER3,
	STEPPER4,
	STEPPER5,
	STEPPER6
}Stepper_t;
typedef struct {
//  speed ramp state we are in.
	ramp_state_t run_state;
//  Direction stepper motor should move.
	unsigned char dir;
//  Counter peroid of timer delay (ARR). At start this value set the accelration rate.
	unsigned int step_delay;
//  step_pos to start deceleration
	unsigned int decel_start;
//  Number of stepp for deceleration(in negative).
	signed int decel_val;
//  Minimum time ARR (max speed)
	signed int min_step_delay;
//  Counter used when accelerateing/decelerateing to calculate step_delay.
	signed int accel_count;
//  Counting steps when moving.
	unsigned int step_count;
//  Keep track of remainder from new_step-delay calculation to increase accuracy
	unsigned int rest;
	unsigned int new_step_delay;// Holds next delay period.
//  Remember the last step delay used when accelerating.
	unsigned int last_accel_delay;

	GPIO_TypeDef* Step_Port;
	GPIO_TypeDef* Dir_Port;
	uint16_t Step_Pin;
	uint16_t Dir_Pin;
	TIM_HandleTypeDef* htim;
} Acceleration_t;

void Accel_Stepper_SetTimer(Acceleration_t *Accel_stepper, TIM_HandleTypeDef* htim);
void Accel_Stepper_SetPin(Acceleration_t *Accel_stepper, GPIO_TypeDef* step_port, uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin);
void Accel_Stepper_Move(Acceleration_t *Accel_stepper, signed int step, unsigned int accel, unsigned int decel, unsigned int rpm);//acc*100
void Accel_Stepper_TIMIT_Handler(Acceleration_t *Accel_stepper);


#endif /* INC_ACCEL_STEPPER_H_ */
