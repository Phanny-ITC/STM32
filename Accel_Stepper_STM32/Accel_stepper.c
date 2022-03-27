/*
 * Accel_stepper.c
 *
 *  Created on: Mar 26, 2022
 *      Author: Phanny
 *      This Library is base on AVR446: Linear speed control of stepper motor
 *      It is now implement with STM32
 *      For document you can find at
 *      https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf
 */
#include "Accel_stepper.h"


Acceleration_t Accel[sizeof(Stepper_t)];
/*
 * Set GPIO for each stepper
 * stepper : Num of whitch stepper use found @ Stepper_t
 * step_port : GPIO port of step pin
 * step_pin : gpio pin number of step pin
 * dir_port : GPIO port of direction pin
 * dir_pin : gpio pin number of direction pin
 */
void Accel_Stepper_SetPin(Stepper_t stepper, GPIO_TypeDef* step_port,
		uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin)
{
	Accel[stepper].Step_Port = step_port;
	Accel[stepper].Dir_Port = dir_port;
	Accel[stepper].Step_Pin = step_pin;
	Accel[stepper].Dir_Pin = dir_pin;
}
/*
 * Set Timer for each motor
 * stepper : Num of which stepper use found @ Stepper_t
 * timer : pointer to timer typedef(Which timer is use for control speed)
 */
void Accel_Stepper_SetTimer(Stepper_t stepper, TIM_HandleTypeDef* timer){
	Accel[stepper].htim = timer;
}
/*
 * Accel_Stepper_TIMIT_Handler
 * stepper : Num of which stepper use found @ Stepper_t
 */
void Accel_Stepper_TIMIT_Handler(Stepper_t stepper){

	unsigned int new_step_delay;// Holds next delay period.
	// Remember the last step delay used when accelerating.
	static int last_accel_delay;
	// Counting steps when moving.
	static unsigned int step_count = 0;
	// Keep track of remainder from new_step-delay calculation to increase accuracy
	static unsigned int rest = 0;
	__HAL_TIM_SET_AUTORELOAD(Accel[stepper].htim, Accel[stepper].step_delay);

	switch(Accel[stepper].run_state) {
		case STOP:
		   	step_count = 0;
		   	rest = 0;
		     // Stop Timer/Counter 1.
		   	HAL_TIM_Base_Stop_IT(Accel[stepper].htim);
//		      status.running = false;
		   	break;
	    case ACCEL:
//		      rc = srd.dir;
//	    	 Generate pulse for stepper driver
	    	HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 1);
			HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 1);
			HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 0);
			HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 0);
			step_count++;
			Accel[stepper].accel_count++;
			new_step_delay = Accel[stepper].step_delay - (((2 * (long)Accel[stepper].step_delay) + rest)/(4 * Accel[stepper].accel_count + 1));
			rest = ((2 * (long)Accel[stepper].step_delay)+rest)%(4 * Accel[stepper].accel_count + 1);
	      // Chech if we should start decelration.
			if(step_count >= Accel[stepper].decel_start) {
				Accel[stepper].accel_count = Accel[stepper].decel_val;
	        	Accel[stepper].run_state = DECEL;
			}
		      // Chech if we hitted max speed.
			else if(new_step_delay <= Accel[stepper].min_step_delay) {
				last_accel_delay = new_step_delay;
				new_step_delay = Accel[stepper].min_step_delay;
				rest = 0;
				Accel[stepper].run_state = RUN;
			}
			break;

	    case RUN:
//		      rc = srd.dir;
//	    	 Generate pulse for stepper driver
			 HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 1);
			 HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 1);
			 HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 0);
			 HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 0);
			 step_count++;
			 new_step_delay = Accel[stepper].min_step_delay;
//	         Check if we should start deceleration.
			 if(step_count >= Accel[stepper].decel_start) {
				 Accel[stepper].accel_count = Accel[stepper].decel_val;
//	         Start deceleration with same delay as accel ended with.
				 new_step_delay = last_accel_delay;
				 Accel[stepper].run_state = DECEL;
			 }
			 break;

	    case DECEL:
//		      rc = srd.dir;
//	    	 Generate pulse for stepper driver
			 HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 1);
			 HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 1);
			 HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 0);
			 HAL_GPIO_WritePin(Accel[stepper].Step_Port, Accel[stepper].Step_Pin, 0);
			 step_count++;
			 Accel[stepper].accel_count++;
			 new_step_delay = Accel[stepper].step_delay + (((2 * (long)Accel[stepper].step_delay) + rest)/(4 * abs(Accel[stepper].accel_count) + 1));
			 rest = ((2 * (long)Accel[stepper].step_delay)+rest)%(4 * (long) abs(Accel[stepper].accel_count) + 1);
//	         Check if we at last step
			 if(Accel[stepper].accel_count >= 0){
				 Accel[stepper].run_state = STOP;
			 }
			 break;
	  }
	  Accel[stepper].step_delay = new_step_delay;
//		  return rc;
}
/*
 * Accel_Stepper_Move
 * stepper : Number of which stepper use found @ Stepper_t
 * step : Number of step to run
 * accel : acceleration
 * decel : deceleration
 * rpm : speed at run state
 */
void Accel_Stepper_Move(Stepper_t stepper, signed int step, unsigned int accel, unsigned int decel, unsigned int rpm)//acc*100
{

	unsigned int max_step_lim; //! Number of steps before we hit max speed.
	unsigned int accel_lim;//! Number of steps before we must start deceleration (if accel does not hit max speed).
	unsigned int speed = 2 * 3.14159 * rpm/60;
//   Set direction from sign on step value.
	if(step < 0){
//    srd.dir = CCW;
		HAL_GPIO_WritePin(Accel[stepper].Dir_Port, Accel[stepper].Dir_Pin, 0);
		step = -step;
	}
	else{
		HAL_GPIO_WritePin(Accel[stepper].Dir_Port, Accel[stepper].Dir_Pin, 1);
//    srd.dir = CW;
	}

//  If moving only 1 step.
	if(step == 1){
//      Move one step...
		Accel[stepper].accel_count = -1;
//      ...in DECEL state.
		Accel[stepper].run_state = DECEL;
//      Just a short delay so main() can act on 'running'.
		Accel[stepper].step_delay = 1000;
//      status.running = TRUE;
		HAL_TIM_Base_Start_IT(Accel[stepper].htim);
	}
//  Only move if number of steps to move is not zero.
	else if(step != 0){
//	   Set max speed limit, by calc min_step_delay to use in timer.
//	   min_step_delay = (alpha / tt)/ w
//	   Accel[stepper].min_step_delay = alpha*t_freq / speed; //speed(rad/s)
//	   ARR=freq*t, where t=60/step/rpm; t here is in second unit (s) step = 6400(1/32)

		Accel[stepper].min_step_delay = (9375/rpm);

//      Set acceleration by calc the first (c0) step delay .
//      step_delay = 1/tt * sqrt(2*alpha/accel)
//      step_delay = ( tfreq*0.676) * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
//      SRD.step_delay = (TIM_FREQ_SCALE * sqrt(A_SQ / accel))/10000;
		Accel[stepper].step_delay = (TIM_FREQ_SCALE * sqrt(A_SQ / accel))/10000;;
//      Find out after how many steps does the speed hit the max speed limit.(step for accel)
//      max_step_lim = speed^2 / (2*alpha*accel)
		max_step_lim = (long)speed*speed*10000/(long)(((long)A_x20000*accel)/100);
//      If we hit max speed limit before 0,5 step it will round to 0.
//      But in practice we need to move at least 1 step to get any speed at all.
		if(max_step_lim == 0){
			max_step_lim = 1;
		}

//      find out after how many steps we must start deceleration.(step before start decel)
//      n1 = (n1+n2)decel / (accel + decel)
		accel_lim = ((long)step*decel) / (accel+decel);
//      We must accelerate at least 1 step before we can start deceleration.
		if(accel_lim == 0){
			accel_lim = 1;
		}

//     Use the limit we hit first to calculate decel.
		if(accel_lim <= max_step_lim){
			Accel[stepper].decel_val = accel_lim - step;//decel_val: step for decel)
		}
		else{
			Accel[stepper].decel_val = -(((long)(max_step_lim*accel))/decel);
		}
//     We must decelrate at least 1 step to stop.
		if(Accel[stepper].decel_val == 0){
			Accel[stepper].decel_val = -1;
		}

//     Find step to start deceleration.
		Accel[stepper].decel_start = step + Accel[stepper].decel_val;

//     If the maximum speed is too High that we don't need to go via acceleration state.
		if(Accel[stepper].step_delay <= Accel[stepper].min_step_delay){
			Accel[stepper].step_delay = Accel[stepper].min_step_delay;
			Accel[stepper].run_state = RUN;
		}
		else{
			Accel[stepper].run_state = ACCEL;
		}

//     Reset counter.
		Accel[stepper].accel_count = 0;
//    status.running = TRUE;
		__HAL_TIM_SET_AUTORELOAD(Accel[stepper].htim, 10000);
		HAL_TIM_Base_Start_IT(Accel[stepper].htim);
	}
}

