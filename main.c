
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#define pos_desired 0 
#define angle_desired 3.1415
#define posvel_desired 0
#define angvel_desired 0
#define opFreq 100 // System Frequency
#define motorVolt 12
#define EnergyDesired 0.5 //0.3924
#define AccelerationGain 2.8
#define AccelSaturationConstant 2.9
#define gravityAccel 9.81
#define m 0.1
#define M 0.135
#define l 0.2
#define Jm 3.26E-08
#define Rm 12.5
#define Kb 0.031
#define Kt 0.031
#define r 0.006
#define b 0.000078
#define c 0.63               
#define I 0.00072
#define M_t 0.136
#define g 9.81


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Variable declaration 
int lqr_count = 0,swingUpCount = 0;
volatile int AngleCount=0,PosCount=0;                                      
volatile uint16_t checkFlag=0; 
double pos_error=0,posvel_error=0,angle_error=0,anglevel_error=0,in_voltage = 0;
double angleMeasured=0,posMeasured=0,absAngleMeasured = 0;
double linSpeed=0,angSpeed=0;
double Ecal=0, Eerror=0;
double accelCalc=0;
double swingUpF=0,swingUpV=0,swingUpV_t=0;
double linAccelMeasured;

// Process measurement values and return speed/acceleration (angular/linear)
double measSpeed(float newVal,int mode)
{	
	static float linOldVal = 0, angOldVal=0, angAccelOld = 0;
	double speedVal;
	
	// mode 0 -> Angular speed 
	if(mode==0)
	{	
		speedVal = (newVal-angOldVal)*opFreq;
		angOldVal = newVal;
		return speedVal;
	}
	
	// mode 1 -> Linear speed 
	else 	if(mode==1)
	{	
		speedVal = (newVal-linOldVal)*opFreq;
		linOldVal = newVal;
		return speedVal;
	}
	
	// mode 2 -> Angular acceleration
	else if(mode==2)
	{
		speedVal = (newVal-angAccelOld)*opFreq;
		angAccelOld = newVal;
		return speedVal;
	}
	
	return 0;
}
/* Energy Sapping Algorithm */
// Signum function for acceleration to get direction
int accelDirection(double angMeas, double angSpeed)
{
	double thetaCos;
	thetaCos = angSpeed*cos(angMeas);
	
	return ((thetaCos > 0) - (thetaCos < 0));
}

// Saturation function for acceleration
double accelSaturation(double accel)
{
	if(accel > AccelSaturationConstant*gravityAccel)
		return AccelSaturationConstant*gravityAccel;
	else if (accel < -1*AccelSaturationConstant*gravityAccel)
		return -1*AccelSaturationConstant*gravityAccel;
}

// Absolute function
double absoluteVal(double val)
{
	return (val > 0 ? val : (-1*val));
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim4);
	
	HAL_TIM_IC_Start_IT (&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT (&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT (&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT (&htim2,TIM_CHANNEL_2);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		 
		  angleMeasured= AngleCount*0.15*3.14/180;
		
		  posMeasured = PosCount*2*3.14*.6*0.01/2400;
		
		// For running system at desired operating frequency
		if(checkFlag)
		{  checkFlag = 0;
			 
			 // Take measurements for current computation cycle
			 angSpeed = measSpeed(angleMeasured,0);
			 linSpeed = measSpeed(posMeasured,1);
			 absAngleMeasured = absoluteVal(angleMeasured);
			 
			 // Check angular and position extremeties to determine mode of operation (LQR/Swing up)
			 // LQR Control
			 if((absAngleMeasured>(2.8) && absAngleMeasured<3.5) && (posMeasured> -0.16 && posMeasured < 0.16))
					{
						HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
						
						//calcution of error
						pos_error = pos_desired - posMeasured;
						angle_error= angle_desired - absAngleMeasured;
						posvel_error = posvel_desired - linSpeed;
						anglevel_error = angvel_desired - angSpeed;
						
						//calcution of input voltage lqr gain * errors and set max and min limits
							
									// in_voltage = ((-335.41*pos_error) + (549.63*angle_error) + (-199.68*posvel_error) + (54.66*anglevel_error));
								//in_voltage = ((-106.0660*pos_error) + (189.1604*angle_error) + (-70.0515*posvel_error) + (20.4840*anglevel_error));
								//	in_voltage = ((-145.7738*pos_error) + (236.8199*angle_error) + (-89.8603*posvel_error) + (25.7226*anglevel_error));
								//	in_voltage = ((-58.3095*pos_error) + (107.3329*angle_error) + (-42.1086*posvel_error) + (12.6607*anglevel_error));
								//	in_voltage = ((-63.2456*pos_error) + (109.6015*angle_error) + (-43.9207*posvel_error) + (13.1102*anglevel_error));
						
						in_voltage = ((-50*pos_error) + (86.4339*angle_error) + (-36.1695*posvel_error) + (10.8640*anglevel_error));
						
						if(in_voltage >=0)
						{
							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
							if(in_voltage>motorVolt)
									 in_voltage= motorVolt ;	
						}
				
						else
						{
							in_voltage = -1*in_voltage;
							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);		
							if(in_voltage>motorVolt)
								 in_voltage= motorVolt ;
						}
					
				// voltage to count conversion 
			   lqr_count= (1000/motorVolt)*in_voltage;
			  
				// reload period of pwm according to count 
				 htim3.Instance->CCR4 = lqr_count;
			
			}																										
		
		// Swing Up
		else if (1)//((posMeasured> -0.12 && posMeasured < 0.12))
		{
			
			Ecal = (0.5*(I+(m*l*l))*angSpeed*angSpeed)+ (m*g*l*(1-cos(angleMeasured)));
			Eerror = Ecal - EnergyDesired;
			accelCalc = accelDirection(angleMeasured, angSpeed)*Eerror;
			accelCalc = accelSaturation(accelCalc);
			accelCalc = accelCalc*AccelerationGain*gravityAccel;
			
			linAccelMeasured = measSpeed(linSpeed,2); 
			
			swingUpF = (M+m)*accelCalc - cos(angleMeasured)*(m*l/(I+(m*l*l)))*(b*angSpeed + m*l*accelCalc*cos(angleMeasured) + m*g*l*sin(angleMeasured)) + c*linSpeed - m*l*angSpeed*angSpeed*sin(angleMeasured);
			swingUpV = ((Rm*r/Kt)*swingUpF + (Kb/r)*linSpeed);
			if(swingUpV >=0)
				{ 
					if(swingUpV < 3)
						swingUpV = 3 ;
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
						if(swingUpV>motorVolt)
				       swingUpV= motorVolt ;
						
				}
				
			   	else
				{
					swingUpV = -1*swingUpV;
					if(swingUpV < 3)
						swingUpV = 3 ;
	         HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);		
						if(swingUpV>motorVolt)
				       swingUpV= motorVolt;
				}

			//voltage to count conversion 
			swingUpCount= (1000/motorVolt)*swingUpV;
			  
			// reload period of pwm according to count 
			htim3.Instance->CCR4 = swingUpCount;	
		}
		
		else
			htim3.Instance->CCR4 = 0;
		
		}
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 41;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 41;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 90;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 839;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD0 PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//Timer for system frequency
uint32_t of_count=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	
	if(htim->Instance==TIM4)
	{  of_count++;
		checkFlag = 1;
	}
  
}


uint32_t InputCaptureValue;
uint32_t inputCaptureVal1,inputCaptureVal2;

//uint16_t old1=0,new1=0,pin9_t,val12=0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	//Updating count as per encoder output for angle and position measurement
	
	//Angle Measurement
	if(htim->Instance == TIM1)
		{

					if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)	
			      
									{   			
											if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)!=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11))									                                                 
										        AngleCount++;
												
											else                               
											      AngleCount--;                             
									}
		

					else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)	
					
									{   			
											if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)==HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9))
													  AngleCount++;
											else
														AngleCount--;
									}
      
									AngleCount = (AngleCount)%2400;
			}
		
			
			// Position Measurement
			else if(htim->Instance == TIM2)
			{
					
					if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)	
									{   			
											if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)!=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1))									
															 PosCount++;
											else   
															 PosCount--;
									}
		

					else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)	
									{   			
											if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))     
															PosCount++;
											else     
															PosCount--;
									}
			}

		}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
