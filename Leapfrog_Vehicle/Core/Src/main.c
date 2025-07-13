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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include <errno.h>
#include <stdarg.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// Include source files

//#include "heartbeat.h"
#include "task.h"
#include "queue.h"
#include "tvc.h"
#include "ACS_PID.h"
#include "processIMU.h"
#include "altimeter.h"
#include "engine_PID.h"
#include "engine.h"
#include "alt_imu_coupling.h"
#include "gps.h"
//#include "printf.h"
//#include "Test_Operations.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 512
#define UART_TIMEOUT 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ACSTask */
osThreadId_t ACSTaskHandle;
const osThreadAttr_t ACSTask_attributes = {
  .name = "ACSTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tvcTask */
osThreadId_t tvcTaskHandle;
const osThreadAttr_t tvcTask_attributes = {
  .name = "tvcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KillSwitchPin */
osThreadId_t KillSwitchPinHandle;
const osThreadAttr_t KillSwitchPin_attributes = {
  .name = "KillSwitchPin",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for engineTask */
osThreadId_t engineTaskHandle;
const osThreadAttr_t engineTask_attributes = {
  .name = "engineTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lidarMutex */
osMutexId_t lidarMutexHandle;
const osMutexAttr_t lidarMutex_attributes = {
  .name = "lidarMutex"
};
/* USER CODE BEGIN PV */

// Define the global instance of STM32Response
STM32Response stm32Response;

// Define system state variables
SubsystemState acsState;
SubsystemState tvcState;
SubsystemState engineState;

// Heartbeat Variables
QueueHandle_t piMessageQueueHandle;


// ACS Variables
uint8_t imuCalibrate = 0;
uint8_t imuCalibrate_Sticky = 0;


// Altimeter Variables
#define ALT_QUEUE_LENGTH 1
#define ALT_QUEUE_ITEM_SIZE sizeof(AltimeterData)
QueueHandle_t AltimeterQueueHandle;


// TVC Variables
//TODO Placeholder queue creation! (TVC_Data needs to be updated in tvc.c!)
#define TVC_QUEUE_LENGTH 1
#define TVC_QUEUE_ITEM_SIZE sizeof(TVC_Data)
QueueHandle_t tvcQueueHandle;


// GPS Variables
// TODO Placeholder queue creation!
#define GPS_QUEUE_LENGTH  1
#define GPS_QUEUE_ITEM_SIZE sizeof(GPS_Data)
QueueHandle_t gpsQueueHandle;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM8_Init(void);
void StartDefaultTask(void *argument);
void StartACSTask(void *argument);
void StartDebugTask(void *argument);
void StartTVCTask(void *argument);
void StartKillSwitchPin(void *argument);
void StartEngineTask(void *argument);

/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void debugOutput(UART_HandleTypeDef *huart);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Custom _putchar for lightweight printf to UART1
void _putchar(char character) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&character, 1, HAL_MAX_DELAY);
}



 uint8_t modeButton = 0; // Used for Blue Button to toggle external interrupt

/* _write function implementation */
int _write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{   // Handle incoming IMU data and restart the UART reception interrupt.
	// This function is automatically called by the HAL library when a UART receive operation is completed (HAL_UART_Receive_IT).
	// It takes a pointer to the UART handle (huart) that triggered the interrupt.

    // Handle overrun errors first
	// IMU specifically seems to hit ORE (overrun) a lot ¯\_(ツ)_/¯
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        printf("UART Overrun Error!\r\n");
    }

	if (huart->Instance == USART6) {
		// Restart UART reception
		HAL_UART_Receive_IT(&huart6, imu1RxBuf, IMU_PACKET_LEN);
	}
	if (huart->Instance == UART4) {
		// Restart UART reception
		HAL_UART_Receive_IT(&huart4, imu2RxBuf, IMU_PACKET_LEN);
	}
	if (huart->Instance == USART3) {
		// Restart UART reception
		HAL_UART_Receive_IT(&huart3, imu3RxBuf, IMU_PACKET_LEN);
	}


}




void debugOutput(UART_HandleTypeDef *huart){ // Debug for code setup
    char uart_buffer[768]; // shared buffer for UART,
    // Full message is too large for buffer.

    // Clear the buffer
    memset(uart_buffer, 0, sizeof(uart_buffer));

    // Make string for printing test data
    int len = snprintf(uart_buffer, sizeof(uart_buffer),
//	    	"\rIMU1 - R: %.2f, P: %.2f, Y: %.2f \r\n"
//	    	"\rIMU2 - R: %.2f, P: %.2f, Y: %.2f \r\n"
//	    	"\rIMU3 - R: %.2f, P: %.2f, Y: %.2f \r\n"
			"\rroll: %.2f, pitch: %.2f, yaw: %.2f \r\n"
			"\rP_pitch: %.2f, I_pitch: %.2f, D_pitch: %.2f \r\n"
			"\rP_roll: %.2f, I_roll: %.2f, D_roll: %.2f \r\n"
			"\rP_yaw: %.2f, I_yaw: %.2f, D_yaw: %.2f \r\n"
			"\rAngVel_X: %.2f, AngVel_Y: %.2f \r\n"
			"\rIntegral_pitch: %.2f, Integral_roll: %.2f, Integral_yaw: %.2f \r\n"
			"\rPR1: %.2f, PR2: %.2f, PR3: %.2f, PR4: %.2f, Y1: %.2f, Y2: %.2f \r\n"
			"\rAltitude: %d, Adjusted Altitude: %0.2f \r\n"
    		"\rEngine PID Output: %.2f, Engine Throttle %%: %d\r\n"
			"\r\n",
//			imu_roll[0], imu_pitch[0], imu_yaw[0],
//			imu_roll[1], imu_pitch[1], imu_yaw[1],
//			imu_roll[2], imu_pitch[2], imu_yaw[2],
			processed_roll_deg, processed_pitch_deg, processed_yaw_deg,
			P_pitch, I_pitch, D_pitch,
			P_roll, I_roll, D_roll,
			P_yaw, I_yaw, D_yaw,
			current_angvel_x_degs, current_angvel_y_degs,
			integral_pitch,	integral_roll, integral_yaw,
			PR1_output, PR2_output, PR3_output, PR4_output, Y1_output, Y2_output,
			(int16_t)latestData.altitude, adjusted_altitude_cm,
			altPIDOut, throttle_percent
    	);

    // Transmit over UART
    HAL_UART_Transmit(huart, (uint8_t *)uart_buffer, len, HAL_MAX_DELAY);
}



// Callback function | every time blue button is pressed, the function is called
// Physical on/off switch... nice!
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	modeButton = modeButton + 1;
	if (modeButton > 1)
	{
		modeButton = 0;
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */


  HAL_Delay(500);    // Short delay to allow systems to finish initializing

  printf("\r\nSystem initialized!\r\n");
  HAL_Delay(20);

  /* IMU HANDLING ------------------------------------------------------------- */
  printf("Starting UART check...\r\n");

  // Start UART reception for IMU data (only after IMU is fully ready)
  if (HAL_UART_Receive_IT(&huart6, imu1RxBuf, IMU_PACKET_LEN) != HAL_OK) {
      printf("IMU1 UART6 Initialization Failed!\r\n");
      Error_Handler();
  }
  else{
	  printf("\rIMU1 USART6 Initialization Complete!\r\n");
  }
  HAL_Delay(10);

  if (HAL_UART_Receive_IT(&huart4, imu2RxBuf, IMU_PACKET_LEN) != HAL_OK) {
        printf("IMU2 UART4 Initialization Failed!\r\n");
        Error_Handler();
  }
  else{
  	  printf("\rIMU2 USART4 Initialization Complete!\r\n");
    }
  HAL_Delay(10);

  if (HAL_UART_Receive_IT(&huart3, imu3RxBuf, IMU_PACKET_LEN) != HAL_OK) {
        printf("IMU3 UART3 Initialization Failed!\r\n");
        Error_Handler();
  }
  else{
  	  printf("\rIMU3 USART3 Initialization Complete! \r\n");
    }


  HAL_Delay(200);



  /* ALTIMITER HANDLING -------------------------------------------------------- */
  printf("\rStarting I2C check... \r\n");


// Initialize Altimeter with desired distance mode
  altimeterInit(&hi2c1, ALTIMETER_SHORT_DISTANCE_MODE);  // Use SHORT mode (for low sensitivity, high precision)
// OR:
// altimeterInit(&hi2c1, ALTIMETER_LONG_DISTANCE_MODE);  // Use LONG mode (for high sensitivity, longer range)


  if (altimeterCheckI2CReady() != HAL_OK) {
      printf("\rAltimeter I2C check failed. LIDAR not responding!\r\n");
      Error_Handler();  // Stop execution if LIDAR is not responding
  } else {
      printf("\rAltimeter Initialized! \r\n");
  }
  HAL_Delay(200);

/* End sensor initialization --------------------------------------------------- */


	/* Start PWM channels for Pitch & Roll EDFs 1-4 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	/* Start PWM for Yaw EDF 5&6, 7&8 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	void SetFanSpeed(uint32_t Fan1, uint32_t Fan2, uint32_t Fan3, uint32_t Fan4, uint32_t Fan5, uint32_t Fan6) {
	    TIM3->CCR1 = Fan1;  // Fan 1
	    TIM3->CCR2 = Fan2;  // Fan 2
	    TIM3->CCR3 = Fan3;  // Fan 3
	    TIM3->CCR4 = Fan4;  // Fan 4
	    TIM4->CCR1 = Fan5;  // Fan 5
	    TIM4->CCR2 = Fan6;  // Fan 6
	}
	        HAL_Delay(1000); // Allow ESCs to initialize
	        SetFanSpeed(50, 50, 50, 50, 50, 50);  // Sets all throttle to 0%
	        HAL_Delay(3000);

//	        printf("\n Startup Test Begins! \r\n");
//
//	        SetFanSpeed(60, 50, 50, 50, 50, 50);  // Fan 1
//	        HAL_Delay(2000);
//
//	        SetFanSpeed(50, 60, 50, 50, 50, 50);  // Fan 2
//	        HAL_Delay(2000);
//
//	        SetFanSpeed(50, 50, 60, 50, 50, 50);  // Fan 3
//	        HAL_Delay(2000);
//
//	        SetFanSpeed(50, 50, 50, 60, 50, 50);  // Fan 4
//	        HAL_Delay(2000);
//
//	        SetFanSpeed(50, 50, 50, 50, 60, 50);  // Fan 5
//	        HAL_Delay(4000);
//
//	        SetFanSpeed(50, 50, 50, 50, 50, 60);  // Fan 6
//	        HAL_Delay(4000);
//
//	        SetFanSpeed(50, 50, 50, 50, 50, 50);  // Reset all throttle to 0%
//
//	        printf("\n Startup Test complete! \r\n");
//
//	        HAL_Delay(2000);

	        printf("\rAll ESCs Initialized! \r\n");
	        HAL_Delay(200);

	/* Start timer interrupt for PID control */
	HAL_TIM_Base_Start_IT(&htim2);


	HAL_Delay(100);

    printf("\rSystem starting... \r\n");

    HAL_Delay(1000);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of lidarMutex */
  lidarMutexHandle = osMutexNew(&lidarMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */


  	// Heartbeat Queue
  	piMessageQueueHandle = xQueueCreate(1, sizeof(STM32Message));
  	if (piMessageQueueHandle == NULL) {
  	    printf("Failed to create piMessage Queue!\r\n");
  	    Error_Handler();
  	}

  	// TVC Queue ... TODO unused for now
  	tvcQueueHandle = xQueueCreate(TVC_QUEUE_LENGTH, TVC_QUEUE_ITEM_SIZE);
  	if (tvcQueueHandle == NULL) {
  	    printf("Failed to create TVC Queue!\r\n");
  	    Error_Handler();
  	}

    // Altimeter Queue ... TODO unused for now
    AltimeterQueueHandle = xQueueCreate(ALT_QUEUE_LENGTH, ALT_QUEUE_ITEM_SIZE);
    if (AltimeterQueueHandle == NULL) {
        printf("\rFailed to create ALTIMETER Queue!\r\n");
        Error_Handler();
    }

  	// GPS Queue ... TODO unused for now
  	 gpsQueueHandle = xQueueCreate(GPS_QUEUE_LENGTH, GPS_QUEUE_ITEM_SIZE);
     if (gpsQueueHandle == NULL) {
         printf("\rFailed to create GPS Queue!\r\n");
         Error_Handler();
     }





  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ACSTask */
  ACSTaskHandle = osThreadNew(StartACSTask, NULL, &ACSTask_attributes);

  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

  /* creation of tvcTask */
  tvcTaskHandle = osThreadNew(StartTVCTask, NULL, &tvcTask_attributes);

  /* creation of KillSwitchPin */
  KillSwitchPinHandle = osThreadNew(StartKillSwitchPin, NULL, &KillSwitchPin_attributes);

  /* creation of engineTask */
  engineTaskHandle = osThreadNew(StartEngineTask, NULL, &engineTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  printf("\rKernel starting...\r\n");
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1677-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1677-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1677-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_Reset_GPIO_Port, IMU_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TVC2_REV_Pin|TVC2_FWD_Pin|TVC1_REV_Pin|TVC1_FWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_Reset_Pin */
  GPIO_InitStruct.Pin = IMU_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TVC2_REV_Pin TVC2_FWD_Pin TVC1_REV_Pin TVC1_FWD_Pin */
  GPIO_InitStruct.Pin = TVC2_REV_Pin|TVC2_FWD_Pin|TVC1_REV_Pin|TVC1_FWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
		const TickType_t xFrequency = pdMS_TO_TICKS(1000);
		const uint8_t timeoutCycles = 10;
		TickType_t xLastWakeTime = xTaskGetTickCount();
	HAL_Delay(20);
	printf("\r\nHeartbeat Initializing... \r\n\n");


	uint8_t timeout_counter = 0; // TODO Also defined in heartbeat.c - clean up!


	STM32Message heartbeat;

	while (1)
	  {
	    vTaskDelayUntil(&xLastWakeTime, xFrequency);


	    // Receive heartbeat from Pi (if available)
	    if (xQueueReceive(piMessageQueueHandle, &heartbeat, 0) == pdTRUE) {
	      timeout_counter = timeoutCycles;

	      // Update system states from Pi command
	      tvcVal1_angular = heartbeat.tvcVal1_angular;
	      tvcVal2_angular = heartbeat.tvcVal2_angular;
	      engine_thrust_p = heartbeat.engine_thrust_p;
	      engine_hover_m = heartbeat.engine_hover_m;

	      // Update states from heartbeat
	      acsState     = (SubsystemState) heartbeat.acs_state;
	      tvcState     = (SubsystemState) heartbeat.tvc_state;
	      engineState  = (SubsystemState) heartbeat.engine_state;
	      engineSafe   = heartbeat.engine_safe;
	      enginePwr    = heartbeat.engine_power;

	      // Also update TVC manual targets
	      tvcManualSetpoint1 = heartbeat.tvcVal1_angular;
	      tvcManualSetpoint2 = heartbeat.tvcVal2_angular;

	      // Optional IMU recalibration trigger
	      if (heartbeat.calibrateIMU > 0) {
	        imuCalibrate_Sticky = 1;
	      }

	    }
	    else if (timeout_counter > 0) {
	      --timeout_counter;
	    }

	    // Safety fallback: disable if timeout reached
	    if (timeout_counter == 0) {
	      acsState = SystemAutomatic;
	      tvcState = SystemDisabled;
	      engineState = SystemDisabled;
	    }

	    // Build heartbeat response
	    stm32Response.heartbeat_counter = timeout_counter;
	    buildHeartbeatPacket(&stm32Response);


	#ifndef DEBUG_PRINTF
	    // TODO Enable heartbeat send once ready
//	    sendHeartbeatPacket(&huart2, &stm32Response);  // encoded heartbeat message
//	    heartbeatDebugOutput(&huart2, &stm32Response); // debug output text

	#endif

	  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartACSTask */
/**
* @brief Function implementing the ACSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartACSTask */
void StartACSTask(void *argument)
{
  /* USER CODE BEGIN StartACSTask */

		const TickType_t xFrequency = pdMS_TO_TICKS(100);
		TickType_t xLastWakeTime = xTaskGetTickCount();

		float EDFReadyState = 50;  // Idle state for ESC
		static bool imuCalibrated = false;


		/* Infinite loop */
		for(;;)
		{
			// Set task to delay until the inputted timespan
			vTaskDelayUntil(&xLastWakeTime,xFrequency);

			// Check that our UARTs are pending, they should always be in busy_rx
			if (huart6.RxState != HAL_UART_STATE_BUSY_RX) {
				__HAL_UART_CLEAR_OREFLAG(&huart6);
				HAL_UART_Receive_IT(&huart6, imu1RxBuf, IMU_PACKET_LEN);
			}
			if (huart4.RxState != HAL_UART_STATE_BUSY_RX) {
				__HAL_UART_CLEAR_OREFLAG(&huart4);
				HAL_UART_Receive_IT(&huart4, imu2RxBuf, IMU_PACKET_LEN);
			}
			if (huart3.RxState != HAL_UART_STATE_BUSY_RX) {
				__HAL_UART_CLEAR_OREFLAG(&huart3);
				HAL_UART_Receive_IT(&huart3, imu3RxBuf, IMU_PACKET_LEN);
			}

			// Calibration check
		if (!imuCalibrated) {
	        // Check if we're in flight mode or at high altitude

	        if (adjusted_altitude_cm > 200) {  // 2 meters = 200 cm
	                printf("\r\n ERROR: Cannot calibrate IMU during flight, or unscheduled descent!\r\n");
	        	}
	        else {
	        	if (calibrateIMU_RTOS() == CALIBRATION_COMPLETE) {
	               printf("\r\n[ACS] Initial IMU Calibration Complete.\r\n");
	               imuCalibrated = true;
	            }
	        }
		}
		else { // Normal ACS Operation


            // Process IMU data here
            processIMUData(imu1RxBuf, 1);
            processIMUData(imu2RxBuf, 2);
            processIMUData(imu3RxBuf, 3);

            // If heartbeat detects a calibration input, run calibration
            // Safety checks to prevent calibration during flight or at high altitude
            if (imuCalibrate_Sticky == 1) {
                // Check if we're in flight mode or at high altitude

                if (adjusted_altitude_cm > 200) {  // 2 meters = 200 cm
                    printf("\r\n ERROR: Cannot calibrate IMU during flight, or unscheduled descent!\r\n");
                    imuCalibrate_Sticky = 0;  // Clear the calibration request
                }
                else {
                    printf("\r\n IMU Calibration started...\r\n");
                    calibrateIMU_RTOS();
                    osDelay(10);
                    imuCalibrate_Sticky = 0; // clear after completion
                }
            }

            // Check IMU data agreement & apply calibration offsets
    		IMUCrossCheck();

    		uint32_t currentTime = HAL_GetTick();

    		// Enable or Disable ACS using button
    		if (modeButton == 0 && acsState != SystemDisabled) {
    		    acsState = SystemDisabled;
    		}
    		else if (modeButton == 1 && acsState != SystemAutomatic) {
    		    acsState = SystemAutomatic;
    		}


    		// Run ACS system if heartbeat command sets "SystemAutomatic" mode
    		if (acsState == SystemAutomatic && (currentTime - lastUpdate >= 100)) {

    		    ACS_PID_Control(); 			// Enable ACS system

    		    lastUpdate = currentTime;
    		}

    		else if (acsState == SystemDisabled) {
    		    // Stop the control loop — put ACS into "idle" state
				TIM3->CCR1 = EDFReadyState;
				TIM3->CCR2 = EDFReadyState;
				TIM3->CCR3 = EDFReadyState;
				TIM3->CCR4 = EDFReadyState;
				TIM4->CCR1 = EDFReadyState;
				TIM4->CCR2 = EDFReadyState;

				// Reset integral values to prevent buildup while disabled
    		    I_pitch = 0;
    		    I_roll  = 0;
    		    I_yaw   = 0;
    		    integral_pitch = 0;
    		    integral_roll  = 0;
    		    integral_yaw   = 0;
    		}
		}

	}
  /* USER CODE END StartACSTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
 * @brief Function implementing the DebugTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartDebugTask */

	// Calculating the time / frequency of task
	const TickType_t xFrequency = pdMS_TO_TICKS(200);
	TickType_t xLastWakeTime = xTaskGetTickCount();

	// Define number of output iterations to skip for debugging monitoring
	int n_iter = 5;

	/* Infinite loop */
	for(int debugIteration=0;true;debugIteration++)
	{
		// Set task to delay until the inputted timespan
		vTaskDelayUntil(&xLastWakeTime,xFrequency);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//outputs debug skipping iterations (easier to read in serial monitor)
		if (debugIteration % n_iter == 0){

			debugOutput(&huart2);
//			heartbeatDebugOutput(&huart2, &stm32Response);


		}

	}
  /* USER CODE END StartDebugTask */
}

/* USER CODE BEGIN Header_StartTVCTask */
/**
* @brief Function implementing the tvcTask thread.
* @param argument: Select TVC_Manual, or TVC_Auto
* @retval None
*/
/* USER CODE END Header_StartTVCTask */
void StartTVCTask(void *argument)
{
  /* USER CODE BEGIN StartTVCTask */

	// Calculating the time / frequency of task
//	const TickType_t xFrequency = pdMS_TO_TICKS(500);
//	TickType_t xLastWakeTime = xTaskGetTickCount();

//	SubsystemState currentMode = TVC_Manual; // Default to manual mode for now

  for(;;)
  {
    osDelay(1);
    if (modeButton == 0){ // Toggled state. Button mode is for testing only
    	Test_TVC(1);
    	Test_TVC(2);
    	Hold(1);
    	Hold(2);
    	}

    if (modeButton == 1){
    	TVC_Manual();

    	}
  }
  /* USER CODE END StartTVCTask */
}

/* USER CODE BEGIN Header_StartKillSwitchPin */
/**
* @brief Function implementing the KillSwitchPin thread.
* @param argument: Not used
* @retval None
*/


/* USER CODE END Header_StartKillSwitchPin */
void StartKillSwitchPin(void *argument)
{
  /* USER CODE BEGIN StartKillSwitchPin */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartKillSwitchPin */
}

/* USER CODE BEGIN Header_StartEngineTask */
/**
* @brief Function implementing the engineTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEngineTask */
void StartEngineTask(void *argument)
{
  /* USER CODE BEGIN StartEngineTask */

	// Initialize engine PID, timing, and constants...
	initEngineControl();

    // Calculating the time / frequency of task
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // Task delay time
    // TODO Restore this to 100 or 200 ms, set at 1000ms for debugging purposes...
    TickType_t xLastWakeTime = xTaskGetTickCount();    // Get the current tick count


    engineState = SystemAutomatic;   // Manual engine state set for testing! //TODO Remove before flight
  /* Infinite loop */
  for(;;)
  {
         // Set task to delay until the inputted timespan
         vTaskDelayUntil(&xLastWakeTime, xFrequency);



         // Check if I2C is ready for the next operation
         if (lidarMeasurementInProgress == false && lidarDistanceReadInProgress == false)
         {
             // Process Altimeter Data
             altimeterReadFrame();

             // Allow process to complete
             //osDelay(1);  // Give time for I2C operations to complete
         }
         else
         {
             if (lidarMeasurementInProgress) {
                 printf("\r\n[LIDAR] Measurement error \r\n");
                 // If this flag is returned, MemTxCpltCallback() is NOT being called
             }
             if (lidarDistanceReadInProgress) {
                 printf("\r\n[LIDAR] Distance read error \r\n");
                 // If this flag is returned, MemRxCpltCallback() is NOT being called
             }
         }

         if (engineState == SystemDisabled) { // TODO remove before flight
             engineState = SystemAutomatic;  // Force auto mode for PID testing
         }
         runEngineControl_Test();

     }



  /* USER CODE END StartEngineTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
	/* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
