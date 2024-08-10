/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h" // Needed for debugging
//#include "i2c.h"
//#include "MPUXX50.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define DELAY_US(us) \
    do { \
         uint32_t start = SysTick->VAL; \
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
         while((start - SysTick->VAL) < ticks); \
    } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//int sensor_read = 0x00000000;
	int position;

	float Kp = 0.083*1.9;//0.120/26;// 0.00325,0.0195,0.5 for 556 Base_speed also works for 656 but slghlty unstable
	float Ki = 0;  //0.05,0.024,0.8 for 700 Base_speed
	float Kd = .83*0.6; //0.03*5;//1.2; //0.5
	float Kr = 0;//do not touch
	int P, I, D, R;
	int lastError = 0;
	int errors[10] = {0,0,0,0,0,0,0,0,0,0};
	int error_sum = 0;
	int last_end = 0;	// 0 -> Left, 1 -> Right
	int last_idle = 0;

	uint32_t basespeed = 700;
	uint32_t maxspeedr = 593+30;
	uint32_t maxspeedl = 593+30;
	uint32_t basespeedr = 550; // as a minimum 125 works
	uint32_t basespeedl = 550;// as a minimum 125 works
	const uint32_t sharp = 0;
	const int ARR = 1;//do nout touch
	const int poser = 2;//do not touch
	int SENSOR_DELAY = 5000;
	int calib_active =0;

	int actives = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
//static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	int _write(int file, char *ptr , int len)
		{
			int DataIdx;
			for(DataIdx=0;DataIdx<len; DataIdx++){
				ITM_SendChar(*ptr++);
			}return len;
		}



	void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	}

	void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	}

	void motor_control (double pos_right, double pos_left)
	{
		if (pos_left < 0 )
		{
			HAL_GPIO_WritePin(AENANBLE_GPIO_Port, AENANBLE_Pin, 0);
			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_2, ARR*pos_left);  //run left motor in reverese
		}
		else
		{
			HAL_GPIO_WritePin(AENANBLE_GPIO_Port, AENANBLE_Pin, 1);
			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_2, ARR*pos_left); // run left motor forward

		}
		if (pos_right < 0 )
		{
			HAL_GPIO_WritePin(BENABLE_GPIO_Port, BENABLE_Pin, 1); //run right motor reverse
			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_1, ARR*pos_right);
		}
		else
		{
			HAL_GPIO_WritePin(BENABLE_GPIO_Port, BENABLE_Pin, 0);
			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_1, ARR*pos_right); // run right motor forward
			//printf("motor control: %lf %lf",pos_left,pos_right);

		}
		printf("Motor : %lf %lf", pos_left, pos_right);

	}



	int QTR16_read() {
	    //uint32_t sensor_read = 0x00000000;
	    int pos = 0;
	    int active = 0;
	    //int last_end = -1;  // Initialize to -1 to indicate no sensors are read yet already declared as global variable
	   // int actives;
	    //int last_idle = 0;
	    int position;


		Set_Pin_Output(SENSOR1_GPIO_Port, SENSOR1_Pin);
		Set_Pin_Output(SENSOR2_GPIO_Port, SENSOR2_Pin);
		Set_Pin_Output(SENSOR3_GPIO_Port, SENSOR3_Pin);
		Set_Pin_Output(SENSOR4_GPIO_Port, SENSOR4_Pin);
		Set_Pin_Output(SENSOR5_GPIO_Port, SENSOR5_Pin);
		Set_Pin_Output(SENSOR6_GPIO_Port, SENSOR6_Pin);
		Set_Pin_Output(SENSOR7_GPIO_Port, SENSOR7_Pin);
		Set_Pin_Output(SENSOR8_GPIO_Port, SENSOR8_Pin);
		Set_Pin_Output(SENSOR9_GPIO_Port, SENSOR9_Pin);
		Set_Pin_Output(SENSOR10_GPIO_Port, SENSOR10_Pin);
		Set_Pin_Output(SENSOR11_GPIO_Port, SENSOR11_Pin);
		Set_Pin_Output(SENSOR12_GPIO_Port, SENSOR12_Pin);
		Set_Pin_Output(SENSOR13_GPIO_Port, SENSOR13_Pin);
		Set_Pin_Output(SENSOR14_GPIO_Port, SENSOR14_Pin);
		Set_Pin_Output(SENSOR15_GPIO_Port, SENSOR15_Pin);
		Set_Pin_Output(SENSOR16_GPIO_Port, SENSOR16_Pin);


		HAL_GPIO_WritePin(SENSOR1_GPIO_Port, SENSOR1_Pin,1);
		HAL_GPIO_WritePin(SENSOR2_GPIO_Port, SENSOR2_Pin,1);
		HAL_GPIO_WritePin(SENSOR3_GPIO_Port, SENSOR3_Pin,1);
		HAL_GPIO_WritePin(SENSOR4_GPIO_Port, SENSOR4_Pin,1);
		HAL_GPIO_WritePin(SENSOR5_GPIO_Port, SENSOR5_Pin,1);
		HAL_GPIO_WritePin(SENSOR6_GPIO_Port, SENSOR6_Pin,1);
		HAL_GPIO_WritePin(SENSOR7_GPIO_Port, SENSOR7_Pin,1);
		HAL_GPIO_WritePin(SENSOR8_GPIO_Port, SENSOR8_Pin,1);
		HAL_GPIO_WritePin(SENSOR9_GPIO_Port, SENSOR9_Pin,1);
		HAL_GPIO_WritePin(SENSOR10_GPIO_Port, SENSOR10_Pin,1);
		HAL_GPIO_WritePin(SENSOR11_GPIO_Port, SENSOR11_Pin,1);
		HAL_GPIO_WritePin(SENSOR12_GPIO_Port, SENSOR12_Pin,1);
		HAL_GPIO_WritePin(SENSOR13_GPIO_Port, SENSOR13_Pin,1);
		HAL_GPIO_WritePin(SENSOR14_GPIO_Port, SENSOR14_Pin,1);
		HAL_GPIO_WritePin(SENSOR15_GPIO_Port, SENSOR15_Pin,1);
		HAL_GPIO_WritePin(SENSOR16_GPIO_Port, SENSOR16_Pin,1);


		DELAY_US(20);

		Set_Pin_Input(SENSOR1_GPIO_Port, SENSOR1_Pin);
		Set_Pin_Input(SENSOR2_GPIO_Port, SENSOR2_Pin);
		Set_Pin_Input(SENSOR3_GPIO_Port, SENSOR3_Pin);
		Set_Pin_Input(SENSOR4_GPIO_Port, SENSOR4_Pin);
		Set_Pin_Input(SENSOR5_GPIO_Port, SENSOR5_Pin);
		Set_Pin_Input(SENSOR6_GPIO_Port, SENSOR6_Pin);
		Set_Pin_Input(SENSOR7_GPIO_Port, SENSOR7_Pin);
		Set_Pin_Input(SENSOR8_GPIO_Port, SENSOR8_Pin);
		Set_Pin_Input(SENSOR9_GPIO_Port, SENSOR9_Pin);
		Set_Pin_Input(SENSOR10_GPIO_Port, SENSOR10_Pin);
		Set_Pin_Input(SENSOR11_GPIO_Port, SENSOR11_Pin);
		Set_Pin_Input(SENSOR12_GPIO_Port, SENSOR12_Pin);
		Set_Pin_Input(SENSOR13_GPIO_Port, SENSOR13_Pin);
		Set_Pin_Input(SENSOR14_GPIO_Port, SENSOR14_Pin);
		Set_Pin_Input(SENSOR15_GPIO_Port, SENSOR15_Pin);
		Set_Pin_Input(SENSOR16_GPIO_Port, SENSOR16_Pin);



		// Threshold
		HAL_Delay(1);
		//DELAY_US(25);
		/*for(int i=0;i<=SENSOR_DELAY;i++)
			DELAY_US(500); */

		 printf("Hello");

	    // Read from 16 sensors
		//printf("Sensor read called");
	    if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin)) {
	        //sensor_read |= 0x00000001;
	       pos += 0;
	        active++;
	        last_end = 1;
	        printf("1");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin)) {
	       // sensor_read |= 0x00000002;0
	        pos += 1000;
	        active++;
	       // last_end = 1;
	        printf("2");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin)) {
	      //  sensor_read |= 0x00000004;
	        pos += 2000;
	        active++;
	        printf("3");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin)) {
	       // sensor_read |= 0x00000008;
	        pos += 3000;
	        active++;
	        printf("4");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin)) {
	        //sensor_read |= 0x00000010;
	        pos += 4000;
	        active++;
	        printf("5");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR6_GPIO_Port, SENSOR6_Pin)) {
	       // sensor_read |= 0x00000020;
	        pos += 5000;
	        active++;
	        printf("6");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR7_GPIO_Port, SENSOR7_Pin)) {
	       // sensor_read |= 0x00000040;
	        pos += 6000;
	        active++;
	        printf("7");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR8_GPIO_Port, SENSOR8_Pin)) {
	      //  sensor_read |= 0x00000080;
	        pos += 7000;
	        active++;
	        printf("8");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR9_GPIO_Port, SENSOR9_Pin)) {
	       // sensor_read |= 0x00000100;
	        pos += 8000;
	        active++;
	        printf("9");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR10_GPIO_Port, SENSOR10_Pin)) {
	      //  sensor_read |= 0x00000200;
	        pos += 9000;
	        active++;
	        printf("10");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR11_GPIO_Port, SENSOR11_Pin)) {
	      //  sensor_read |= 0x00000400;
	        pos += 10000;
	        active++;
	        printf("11");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR12_GPIO_Port, SENSOR12_Pin)) {
	      //  sensor_read |= 0x00000800;
	        pos += 11000;
	        active++;
	        printf("12");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR13_GPIO_Port, SENSOR13_Pin)) {
	      //  sensor_read |= 0x00001000;
	        pos += 12000;
	        active++;
	        printf("13");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR14_GPIO_Port, SENSOR14_Pin)) {
	       // sensor_read |= 0x00002000;
	        pos += 13000;
	        active++;
	        printf("14");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR15_GPIO_Port, SENSOR15_Pin)) {
	      // sensor_read |= 0x00004000;
	        pos += 14000;
	        active++;
	       // last_end = 0;
	        printf("15");
	    }
	    if (HAL_GPIO_ReadPin(SENSOR16_GPIO_Port, SENSOR16_Pin)) {
	       // sensor_read |= 0x00008000;
	        pos += 15000;
	        active++;
	        last_end = 0;
	        printf("16");
	    }

	    // Calculate the average position
	    actives = active;
	    position = (active != 0) ? pos / active : -1;  // Handle division by zero

	    if (actives == 0)
	        last_idle++;
	    else
	        last_idle = 0;

	    printf("helloo\n");

	   // HAL_Delay(10);

	    return position;
	    //return 0;



	}


	void sharp_turn () {

			if (last_idle < 10)   //last idle increments while no sensor reads a line and causes bot to turn to find line
			{						// set to handle curves
				if (last_end == 0) //last end tells which side of sensor array last saw the line
					motor_control(-30, 1000);
				else
					motor_control(1000, -30);
				//tune the values to handle the curves
			}
			else //set to handle sharp turns
			{
				if (last_end == 0)
					motor_control(-930, 1000);// tune the values to handle the sharp turns
				else
					motor_control(1000, -930);
			}
			printf("Sharp turn");
		}

	void forward_brake(int pos_right, int pos_left)
	{
		if (actives == 0)
			sharp_turn();
		else
		  motor_control(pos_right, pos_left);
	}
/*
	void past_errors (int error)
	{
	  for (int i = 9; i > 0; i--)
		  errors[i] = errors[i-1];
	  errors[0] = error;
	}

	int errors_sum (int index, int abs)
	{
	  int sum = 0;
	  for (int i = 0; i < index; i++)
	  {
		if ((abs == 1) &( errors[i] < 0))
		  sum += -errors[i];
		else
		  sum += errors[i];
	  }
	  return sum;
	}//*/

	void PID_control() {
		uint32_t position = QTR16_read();
	  int error = (7500) - position;  // needs to be changed for 16 array?
		//past_errors(error);



	  P = error;
	  I = 0;//errors_sum(5, 0);
	  D = error - lastError;
	  R = 0;//errors_sum(5, 1);
	  lastError = error;


	  int motorspeed = P*Kp + I*Ki + D*Kd;

	  int motorspeedl = basespeedl + motorspeed - R*Kr;
	  int motorspeedr = basespeedr - motorspeed - R*Kr;

	  if (motorspeedl > maxspeedl){   //implement antiwindup for potential speed increase and acuracy?
		motorspeedl = maxspeedl;
	  }
	  if (motorspeedr > maxspeedr){
		motorspeedr = maxspeedr;
	  }
	  //printf("Motor SPEEDS : %d %d", motorspeedl, motorspeedr);

		forward_brake(motorspeedr, motorspeedl);
	}


	void Sensor_Calib(){
		//SENSOR_DELAY =7000;
		//calib_active = 0;
		while(calib_active <8){
			//printf("hi");
			calib_active = 0;
		Set_Pin_Output(SENSOR1_GPIO_Port, SENSOR1_Pin);
				Set_Pin_Output(SENSOR2_GPIO_Port, SENSOR2_Pin);
				Set_Pin_Output(SENSOR3_GPIO_Port, SENSOR3_Pin);
				Set_Pin_Output(SENSOR4_GPIO_Port, SENSOR4_Pin);
				Set_Pin_Output(SENSOR5_GPIO_Port, SENSOR5_Pin);
				Set_Pin_Output(SENSOR6_GPIO_Port, SENSOR6_Pin);
				Set_Pin_Output(SENSOR7_GPIO_Port, SENSOR7_Pin);
				Set_Pin_Output(SENSOR8_GPIO_Port, SENSOR8_Pin);
				Set_Pin_Output(SENSOR9_GPIO_Port, SENSOR9_Pin);
				Set_Pin_Output(SENSOR10_GPIO_Port, SENSOR10_Pin);
				Set_Pin_Output(SENSOR11_GPIO_Port, SENSOR11_Pin);
				Set_Pin_Output(SENSOR12_GPIO_Port, SENSOR12_Pin);
				Set_Pin_Output(SENSOR13_GPIO_Port, SENSOR13_Pin);
				Set_Pin_Output(SENSOR14_GPIO_Port, SENSOR14_Pin);
				Set_Pin_Output(SENSOR15_GPIO_Port, SENSOR15_Pin);
				Set_Pin_Output(SENSOR16_GPIO_Port, SENSOR16_Pin);


				HAL_GPIO_WritePin(SENSOR1_GPIO_Port, SENSOR1_Pin,1);
				HAL_GPIO_WritePin(SENSOR2_GPIO_Port, SENSOR2_Pin,1);
				HAL_GPIO_WritePin(SENSOR3_GPIO_Port, SENSOR3_Pin,1);
				HAL_GPIO_WritePin(SENSOR4_GPIO_Port, SENSOR4_Pin,1);
				HAL_GPIO_WritePin(SENSOR5_GPIO_Port, SENSOR5_Pin,1);
				HAL_GPIO_WritePin(SENSOR6_GPIO_Port, SENSOR6_Pin,1);
				HAL_GPIO_WritePin(SENSOR7_GPIO_Port, SENSOR7_Pin,1);
				HAL_GPIO_WritePin(SENSOR8_GPIO_Port, SENSOR8_Pin,1);
				HAL_GPIO_WritePin(SENSOR9_GPIO_Port, SENSOR9_Pin,1);
				HAL_GPIO_WritePin(SENSOR10_GPIO_Port, SENSOR10_Pin,1);
				HAL_GPIO_WritePin(SENSOR11_GPIO_Port, SENSOR11_Pin,1);
				HAL_GPIO_WritePin(SENSOR12_GPIO_Port, SENSOR12_Pin,1);
				HAL_GPIO_WritePin(SENSOR13_GPIO_Port, SENSOR13_Pin,1);
				HAL_GPIO_WritePin(SENSOR14_GPIO_Port, SENSOR14_Pin,1);
				HAL_GPIO_WritePin(SENSOR15_GPIO_Port, SENSOR15_Pin,1);
				HAL_GPIO_WritePin(SENSOR16_GPIO_Port, SENSOR16_Pin,1);


				DELAY_US(15);

				Set_Pin_Input(SENSOR1_GPIO_Port, SENSOR1_Pin);
				Set_Pin_Input(SENSOR2_GPIO_Port, SENSOR2_Pin);
				Set_Pin_Input(SENSOR3_GPIO_Port, SENSOR3_Pin);
				Set_Pin_Input(SENSOR4_GPIO_Port, SENSOR4_Pin);
				Set_Pin_Input(SENSOR5_GPIO_Port, SENSOR5_Pin);
				Set_Pin_Input(SENSOR6_GPIO_Port, SENSOR6_Pin);
				Set_Pin_Input(SENSOR7_GPIO_Port, SENSOR7_Pin);
				Set_Pin_Input(SENSOR8_GPIO_Port, SENSOR8_Pin);
				Set_Pin_Input(SENSOR9_GPIO_Port, SENSOR9_Pin);
				Set_Pin_Input(SENSOR10_GPIO_Port, SENSOR10_Pin);
				Set_Pin_Input(SENSOR11_GPIO_Port, SENSOR11_Pin);
				Set_Pin_Input(SENSOR12_GPIO_Port, SENSOR12_Pin);
				Set_Pin_Input(SENSOR13_GPIO_Port, SENSOR13_Pin);
				Set_Pin_Input(SENSOR14_GPIO_Port, SENSOR14_Pin);
				Set_Pin_Input(SENSOR15_GPIO_Port, SENSOR15_Pin);
				Set_Pin_Input(SENSOR16_GPIO_Port, SENSOR16_Pin);

				DELAY_US(SENSOR_DELAY);
						if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin)) {

							calib_active++;
					        printf("1");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin)) {
					        calib_active++;
					        printf("2");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin)) {
					    	calib_active++;;
					        printf("3");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin)) {
					    	calib_active++;
					        printf("4");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin)) {
					    	calib_active++;
					        printf("5");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR6_GPIO_Port, SENSOR6_Pin)) {
					    	calib_active++;
					        printf("6");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR7_GPIO_Port, SENSOR7_Pin)) {
					    	calib_active++;
					        printf("7");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR8_GPIO_Port, SENSOR8_Pin)) {
					    	calib_active++;
					        printf("8");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR9_GPIO_Port, SENSOR9_Pin)) {
					        calib_active++;
					        printf("9");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR10_GPIO_Port, SENSOR10_Pin)) {
					    	calib_active++;
					        printf("10");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR11_GPIO_Port, SENSOR11_Pin)) {
					    	calib_active++;
					        printf("11");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR12_GPIO_Port, SENSOR12_Pin)) {
					    	calib_active++;
					        printf("12");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR13_GPIO_Port, SENSOR13_Pin)) {
					    	calib_active++;
					        printf("13");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR14_GPIO_Port, SENSOR14_Pin)) {
					    	calib_active++;
					        printf("14");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR15_GPIO_Port, SENSOR15_Pin)) {
					    	calib_active++;
					        printf("15");
					    }
					    if (HAL_GPIO_ReadPin(SENSOR16_GPIO_Port, SENSOR16_Pin)) {
					    	//calib_active++;
					        printf("16");
					    }

					    printf("\n%d\n",SENSOR_DELAY--);
		}


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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
 // HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);



  /* USER CODE END 2 */
  /*
  if (MPU_begin(&hi2c1, AD0_LOW, AFSR_4G, GFSR_500DPS, 0.98, 0.004) == 1)
    {
      //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
	  printf("Hi");

    }
  else
    {
      printf("ERROR!");
      while (1)
      {
        printf("Blocking");
        HAL_Delay(500);
      }
    }
  printf("Caliberating");
  MPU_calibrateGyro(&hi2c1, 1500);
  /* Infinite loop */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 //QTR16_read();
	  //PID_control();
	  printf("Hi");

	  //Sensor_Calib();
	  //printf("hi\n");
	  //printf("delay %d\n",SENSOR_DELAY);
	  //HAL_GPIO_WritePin(BENABLE_GPIO_Port, BENABLE_Pin, 0);
	  //__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1023);
	  //HAL_GPIO_WritePin(AENANBLE_GPIO_Port, AENANBLE_Pin, 1);
	  //__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1023);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1023;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin_Pin|AENANBLE_Pin|BENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIp4_GPIO_Port, DIp4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin_Pin AENANBLE_Pin BENABLE_Pin */
  GPIO_InitStruct.Pin = LED_Pin_Pin|AENANBLE_Pin|BENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP2_Pin SENSOR16_Pin SENSOR8_Pin SENSOR7_Pin
                           SENSOR6_Pin SENSOR5_Pin SENSOR4_Pin SENSOR3_Pin */
  GPIO_InitStruct.Pin = DIP2_Pin|SENSOR16_Pin|SENSOR8_Pin|SENSOR7_Pin
                          |SENSOR6_Pin|SENSOR5_Pin|SENSOR4_Pin|SENSOR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR11_Pin SENSOR13_Pin DP_Pin SENSOR15_Pin
                           SENSOR14_Pin SENSOR12_Pin SENSOR10_Pin SENSOR9_Pin
                           SENSOR2_Pin SENSOR1_Pin DIP3_Pin */
  GPIO_InitStruct.Pin = SENSOR11_Pin|SENSOR13_Pin|DP_Pin|SENSOR15_Pin
                          |SENSOR14_Pin|SENSOR12_Pin|SENSOR10_Pin|SENSOR9_Pin
                          |SENSOR2_Pin|SENSOR1_Pin|DIP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIp4_Pin */
  GPIO_InitStruct.Pin = DIp4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIp4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
