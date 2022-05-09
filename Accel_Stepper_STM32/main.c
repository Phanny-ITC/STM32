/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "tim.h"
//#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "S_curve.h"
//#include "stdio.h"
#include "Accel_stepper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*Declaire acceleraation for each stepper*/
Acceleration_t Stepper1;
Acceleration_t Stepper2;
/*
.
.
Acceleration_t Stepper#n
*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void stepper_set_rpm (uint16_t rpm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void stepper_set_rpm (uint16_t rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	//  stepper2:psc15+1,period2000-1:2500hz,400us
	//	  						1000-1:5000hz,200us
	//	  ARR=48M*t/(15+1), where t=60/step/rpm;t here is in second unit (s) step = 2*6400(1/32)
	uint16_t arr = (uint16_t) (9375/rpm) - 1;//9375 is range/resolution for min rpm
	__HAL_TIM_SET_AUTORELOAD(&htim14, arr);
//	(&htim14)->Instance->ARR = (arr);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
//  stepper_motor_set_param(MOTOR_X, 100);
//	  HAL_TIM_Base_Start_IT(&htim14);
  Accel_Stepper_SetPin(&Stepper1, step_GPIO_Port, step_Pin, dir_GPIO_Port, dir_Pin);
  Accel_Stepper_SetTimer(&Stepper1, &htim14);
  Accel_Stepper_Move(&Stepper1, 640000, 5000, 5000, 1000);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  stepper_motor_set_param(MOTOR_X, fr);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	  if(htim->Instance == TIM14){//stepper3
		  Accel_Stepper_TIMIT_Handler(&Stepper1);
//		 flag = 1;
//			if (angleZ < actual_angleZ){
//				 HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, 1);
//				 if(itr<20000){
//					 u_stp2 = 5+ h_stp2;
//					 h_stp2 +=0.02;
//					 itr++;
//				 }
//				 if((actual_angleZ-angleZ) <= 20000){//20% of 6cm
//				  		sp_stp2 = u_stp2 - h1_stp2;
//				  		h_stp2 += 0.003;
//				 }else sp_stp2 = u_stp2;
////				 HAL_GPIO_TogglePin(step_GPIO_Port, step_Pin);
//				 HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, 1);
//				 HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, 1);
//				 HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, 0);
//				 HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, 0);
//				 state_step2 =! state_step2;
//				 if(state_step2==0) actual_angleZ--;
//
//				 stepper_set_rpm((uint16_t) sp_stp2);
//
//			}
//			else if (angleZ > actual_angleZ){
//				 HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, 0);
//				 if(itr<20000){
//					  //Ts = micro_second/1000; //sample time
//	//				 u_stp2 = 0.0005 * itr;
//	//				 sp_stp2 = 45 * (1/(1 + exp(-a_stp2*(u_stp2-c_stp2))));     // S-curve acceleration
//					 itr++;
//					 u_stp2 = 5 + h_stp2;
//					 h_stp2 +=0.02;
//				 }
//				 if((angleZ-actual_angleZ) <= 20000){//20% of 6cm
//					  sp_stp2 = u_stp2 - h1_stp2;
//					  h_stp2 += 0.003;
//				 }else sp_stp2 = u_stp2;
////				 HAL_GPIO_TogglePin(step_GPIO_Port, step_Pin);
//				 HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, 1);
//				 HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, 1);
//				 HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, 0);
//				 HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, 0);
//				 state_step2 =! state_step2;
//				 if(state_step2==0) actual_angleZ++;
//				 stepper_set_rpm((uint32_t) sp_stp2);
//			 }else{
//				 actual_angleZ = angleZ;
//				 state_step2=0;
//				 itr=0;
//				 u_stp2 = 0;
//				 h_stp2  = 0;
//				 h1_stp2  = 0;
//				 sp_stp2 = 60;
//			 }
	  }
}
//static unsigned long my_sqrt(unsigned long x)
//{
//  register unsigned long xr;  // result register
//  register unsigned long q2;  // scan-bit register
//  register unsigned char f;   // flag (one bit)
//
//  xr = 0;                     // clear result
//  q2 = 0x40000000L;           // higest possible result bit
//  do
//  {
//    if((xr + q2) <= x)
//    {
//      x -= xr + q2;
//      f = 1;                  // set flag
//    }
//    else{
//      f = 0;                  // clear flag
//    }
//    xr >>= 1;
//    if(f){
//      xr += q2;               // test flag
//    }
//  } while(q2 >>= 2);          // shift twice
//  if(xr < x){
//    return xr +1;             // add for rounding
//  }
//  else{
//    return xr;
//  }
//}

/*! \brief Find minimum value.
 *
 *  Returns the smallest value.
 *
 *  \return  Min(x,y).
 */
unsigned int min(unsigned int x, unsigned int y)
{
  if(x < y){
    return x;
  }
  else{
    return y;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
