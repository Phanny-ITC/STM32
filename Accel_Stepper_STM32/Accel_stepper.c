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


//Acceleration_t Accel[sizeof(Stepper_t)];
/*
 * Set GPIO for each stepper
 * Accel_stepper :
 * step_port : GPIO port of step pin
 * step_pin : gpio pin number of step pin
 * dir_port : GPIO port of direction pin
 * dir_pin : gpio pin number of direction pin
 */
void Accel_Stepper_SetPin(Acceleration_t* Accel_stepper, GPIO_TypeDef* step_port,
		uint16_t step_pin, GPIO_TypeDef* dir_port, uint16_t dir_pin)
{
	Accel_stepper->Step_Port = step_port;
	Accel_stepper->Step_Pin = step_pin;
	Accel_stepper->Dir_Pin = dir_pin;
	Accel_stepper->Dir_Port = dir_port;
}
/*
 * Set Timer for each motor
 * stepper : Num of which stepper use found @ Stepper_t
 * timer : pointer to timer typedef(Which timer is use for control speed)
 */
void Accel_Stepper_SetTimer(Acceleration_t *Accel_stepper, TIM_HandleTypeDef* timer){
	Accel_stepper->htim = timer;
}
/*
 * Accel_Stepper_TIMIT_Handler
 * stepper : Num of which stepper use found @ Stepper_t
 */
void Accel_Stepper_TIMIT_Handler(Acceleration_t *Accel_stepper){

	__HAL_TIM_SET_AUTORELOAD(Accel_stepper->htim, Accel_stepper->step_delay);

	switch(Accel_stepper->run_state) {
		case STOP:
			Accel_stepper->step_count = 0;
			Accel_stepper->rest = 0;
		     // Stop Timer/Counter 1.
		   	HAL_TIM_Base_Stop_IT(Accel_stepper->htim);
//		      status.running = false;
		   	break;
	    case ACCEL:
//		      rc = srd.dir;
//	    	 Generate pulse for stepper driver
//	    	HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 1);
//			HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 1);
//			HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 1);
//			HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 0);
//			HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 0);
//			HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 0);
	    	HAL_GPIO_TogglePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin);
	    	Accel_stepper->step_count++;
			Accel_stepper->accel_count++;
			Accel_stepper->new_step_delay = Accel_stepper->step_delay - (((2 * (long)Accel_stepper->step_delay) + Accel_stepper->rest)/(4 * Accel_stepper->accel_count + 1));
			Accel_stepper->rest = ((2 * (long)Accel_stepper->step_delay)+Accel_stepper->rest)%(4 * Accel_stepper->accel_count + 1);
	      // Chech if we should start decelration.
			if(Accel_stepper->step_count >= Accel_stepper->decel_start) {
				Accel_stepper->accel_count = Accel_stepper->decel_val;
				Accel_stepper->run_state = DECEL;
			}
		      // Chech if we hitted max speed.
			else if(Accel_stepper->new_step_delay <= Accel_stepper->min_step_delay) {
				Accel_stepper->last_accel_delay = Accel_stepper->new_step_delay;
				Accel_stepper->new_step_delay = Accel_stepper->min_step_delay;
				Accel_stepper->rest = 0;
				Accel_stepper->run_state = RUN;
			}
			break;

	    case RUN:
//		      rc = srd.dir;
//	    	 Generate pulse for stepper driver
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 1);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 1);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 1);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 0);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 0);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 0);
	    	HAL_GPIO_TogglePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin);
	    	Accel_stepper->step_count++;
	    	Accel_stepper->new_step_delay = Accel_stepper->min_step_delay;
//	         Check if we should start deceleration.
			 if(Accel_stepper->step_count >= Accel_stepper->decel_start) {
				 Accel_stepper->accel_count = Accel_stepper->decel_val;
//	         Start deceleration with same delay as accel ended with.
				 Accel_stepper->new_step_delay = Accel_stepper->last_accel_delay;
				 Accel_stepper->run_state = DECEL;
			 }
			 break;

	    case DECEL:
//		      rc = srd.dir;
//	    	 Generate pulse for stepper driver
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 1);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 1);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 1);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 0);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 0);
//			 HAL_GPIO_WritePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin, 0);
	    	HAL_GPIO_TogglePin(Accel_stepper->Step_Port, Accel_stepper->Step_Pin);
	    	Accel_stepper->step_count++;
			 Accel_stepper->accel_count++;
			 Accel_stepper->new_step_delay = Accel_stepper->step_delay + (((2 * (long)Accel_stepper->step_delay) + Accel_stepper->rest)/(4 * abs(Accel_stepper->accel_count) + 1));
			 Accel_stepper->rest = ((2 * (long)Accel_stepper->step_delay)+Accel_stepper->rest)%(4 * (long) abs(Accel_stepper->accel_count) + 1);
//	         Check if we at last step
			 if(Accel_stepper->accel_count >= 0){
				 Accel_stepper->run_state = STOP;
			 }
			 break;
	  }
	 Accel_stepper->step_delay = Accel_stepper->new_step_delay;
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
void Accel_Stepper_Move(Acceleration_t *Accel_stepper, signed int step, unsigned int accel, unsigned int decel, unsigned int rpm)//acc*100
{

	unsigned int max_step_lim; //! Number of steps before we hit max speed.
	unsigned int accel_lim;//! Number of steps before we must start deceleration (if accel does not hit max speed).
	unsigned int speed = 2 * 3.14159 * rpm/60;
	Accel_stepper->step_count = 0;
//   Set direction from sign on step value.
	if(step < 0){
//    srd.dir = CCW;
		HAL_GPIO_WritePin(Accel_stepper->Dir_Port, Accel_stepper->Dir_Pin, 0);
		step = -step;
	}
	else{
		HAL_GPIO_WritePin(Accel_stepper->Dir_Port, Accel_stepper->Dir_Pin, 1);
//    srd.dir = CW;
	}

//  If moving only 1 step.
	if(step == 1){

//      Move one step...
		Accel_stepper->accel_count = -1;
//      ...in DECEL state.
		Accel_stepper->run_state = DECEL;
//      Just a short delay so main() can act on 'running'.
		Accel_stepper->step_delay = 1000;
//      status.running = TRUE;
		HAL_TIM_Base_Start_IT(Accel_stepper->htim);
	}
//  Only move if number of steps to move is not zero.
	else if(step != 0){
//	   Set max speed limit, by calc min_step_delay to use in timer.
//	   min_step_delay = (alpha / tt)/ w
//	   Accel[stepper].min_step_delay = alpha*t_freq / speed; //speed(rad/s)
//	   ARR=freq*t, where t=60/step/rpm; t here is in second unit (s) step = 6400(1/32)

		Accel_stepper->min_step_delay = (9375/rpm);

//      Set acceleration by calc the first (c0) step delay .
//      step_delay = 1/tt * sqrt(2*alpha/accel)
//      step_delay = ( tfreq*0.676) * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
//      SRD.step_delay = (TIM_FREQ_SCALE * sqrt(A_SQ / accel))/10000;
		Accel_stepper->step_delay = (TIM_FREQ_SCALE * sqrt(A_SQ / accel))/10000;;
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
			Accel_stepper->decel_val = accel_lim - step;//decel_val: step for decel)
		}
		else{
			Accel_stepper->decel_val = -(((long)(max_step_lim*accel))/decel);
		}
//     We must decelrate at least 1 step to stop.
		if(Accel_stepper->decel_val == 0){
			Accel_stepper->decel_val = -1;
		}

//     Find step to start deceleration.
		Accel_stepper->decel_start = step + Accel_stepper->decel_val;

//     If the maximum speed is too High that we don't need to go via acceleration state.
		if(Accel_stepper->step_delay <= Accel_stepper->min_step_delay){
			Accel_stepper->step_delay = Accel_stepper->min_step_delay;
			Accel_stepper->run_state = RUN;
		}
		else{
			Accel_stepper->run_state = ACCEL;
		}

//     Reset counter.
		Accel_stepper->accel_count = 0;
//    status.running = TRUE;
		__HAL_TIM_SET_AUTORELOAD(Accel_stepper->htim, 1000);
		HAL_TIM_Base_Start_IT(Accel_stepper->htim);
	}
}

