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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Configuration parameters

#define STEPS_PER_REV 6400        // Steps per revolution (including microstepping)
#define LEAD_SCREW_PITCH 2        // Lead screw pitch in mm
#define MICROSTEPS 43             // Microstepping factor
#define TIMER_CLOCK_FREQ 72000000 // Timer clock frequency (72 MHz)

// Calculate steps per mm
// #define STEPS_PER_MM (STEPS_PER_REV / LEAD_SCREW_PITCH)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* Definitions for StateHandler */
osThreadId_t StateHandlerHandle;
const osThreadAttr_t StateHandler_attributes = {
    .name = "StateHandler",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime7, // Highest Priority
};

/* Definitions for TempControl */
osThreadId_t TempControlHandle;
const osThreadAttr_t TempControl_attributes = {
    .name = "TempControl",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal7, // Medium-High Priority
};

/* Definitions for Actuations */
osThreadId_t ActuationsHandle;
const osThreadAttr_t Actuations_attributes = {
    .name = "Actuations",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal7, // Medium Priority
};

/* Definitions for trans_distance */
osThreadId_t trans_distanceHandle;
const osThreadAttr_t trans_distance_attributes = {
    .name = "trans_distance",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal7, // Lowest Priority
};

/* Definitions for usbData */
osMessageQueueId_t usbDataHandle;
const osMessageQueueAttr_t usbData_attributes = {
    .name = "usbData"};
/* USER CODE BEGIN PV */
osMessageQueueId_t sendtempHandle;
const osMessageQueueAttr_t sendtemp_attributes = {
    .name = "sendtemp"};
osMessageQueueId_t sendforceHandle;
const osMessageQueueAttr_t sendforce_attributes = {
    .name = "sendforce"};

// osMessageQueueId_t sendlengthHandle;
// const osMessageQueueAttr_t sendlength_attributes = {
//     .name = "sendlength"};
// osMessageQueueId_t sendspeedHandle;
// const osMessageQueueAttr_t sendspeed_attributes = {
//     .name = "sendspeed"};

/* Application States */
typedef enum
{
  sNone,
  sProcess,
  sManual,
  sSaveSettings,
  sHoming,
  sTransmitForce,
} defstate;

/* Application Events */
typedef enum
{
  pNone,
  pFillWater,
  pWaterOff,
  pEmptyWater,
  pTempControlOn,
  pTempControlOff,
  pHoming,
  pmStart,
  pmReverse,
  pmDirVal,
  pStop,
  pPause,
  pReset,
  // pRepeat
} e_defprocess;

typedef enum
{
  uNone,
  uTempControlON,
  uTempControlOFF,
  uMotorStartf,
  uMotorStartb,
  uMotorStop,
  uValveOpen1,
  uValveOpen2,
  uValveClose1,
  uValveClose2
} e_defmanual;

// typedef enum
//{
//  amNone,
//  amStart,
//  amPause,
//  amStop,
//  amDirVal,
//  amReset
// }e_defactuation;

typedef enum
{
  hmNone,
  hmStartpwm,
  hmStopext,
  hmStoptim,
} e_defhoming;

typedef struct
{
  int x;
  e_defprocess xprocess;
  e_defmanual xmanual;
  e_defhoming xhoming;
} event;

// From USB
// volatile uint16_t xdist , xspeed;

//------COM PORT Buffer-----------------------------
volatile uint8_t buff[35];
const char *mfeedback = "*MAN:RES#";
const char *sfeedback = "*PRS:STR#";
const char *pfeedback = "*PRS:PUS#";
const char *rfeedback = "*PRS:HOM#";
const char *nfeedback = "*PRS:RED#";
const char *hfeedback = "*PRS:HET#";
//-------for ADC DMA(FORCE)--------------------------

//-------for user data handling-----------------------
// const char PROCESS[] = "P";
// const char START[] = "1:1";
// const char PAUSE[] = "1:2";
// const char REST[] = "1:3";
// const char LOL[] = "3:5";

// const char MMSRF[] = "MMSRF";
// const char MMSPF[] = "MMSPF";
// const char MMSRB[] = "MMSRB";
// const char MMSPB[] = "MMSPB";
// const char MV1SR[] = "MV1SR";
// const char MV1SP[] = "MV1SP";
// const char MV2SR[] = "MV2SR";
// const char MV2SP[] = "MV2SP";
// const char MHSR[] = "MHSR";
// const char MHSP[] = "MHSP";

//---------for Motor----------------------------------
volatile uint32_t Interrupt_Tim1 = 0;
volatile uint32_t Interrupt_Tim5 = 0;
volatile uint32_t Interrupt_Tim4 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
void StartStateHandler(void *argument);
void StartActuations(void *argument);
void StartTempControl(void *argument);
void trans_distance(void *argument);

/* USER CODE BEGIN PFP */
void stateInit(defstate *const state, event *const e);

// pointer to state functions

// State Machine
void statemachine(defstate *const state, event *const e);
void resetstatemachine(defstate *const state, event *const e);
void saveconditionalstate(defstate const *const state, event const *const e);
void savecurrentstate(defstate const *const state, event const *const e);

// State Finder//no enum assigned, controlled by flags so choose flags wisely
typedef void (*statefinder_t)(defstate *const state, event *const e);

void processEXTinthommin(defstate *const state, event *const e);
void processEXTinthommax(defstate *const state, event *const e);
void processUSBval(defstate *const state, event *const e);
void processTIMintdir(defstate *const state, event *const e);
void processTIMintking(defstate *const state, event *const e);
void processTIMinthom(defstate *const state, event *const e);
void processEXTintWaterIn(defstate *const state, event *const e);
void processEXTintWaterOut(defstate *const state, event *const e);
void processinternalH2P(defstate *const state, event *const e);
void processinternalPh2H(defstate *const state, event *const e);
void processinternalPr2H(defstate *const state, event *const e);
void processinternalW2T(defstate *const state, event *const e);
void processinternalT2H(defstate *const state, event *const e);
void processexternalT2S(defstate *const state, event *const e);
void processexternalF2S(defstate *const state, event *const e);
void processexternalD2S(defstate *const state, event *const e);
void processTIM2SP(defstate *const state, event *const e);
void processinternalPC2R(defstate *const state, event *const e);

statefinder_t statefinder[] = {
    NULL,
    processEXTinthommin,   // 1
    processEXTinthommax,   // 2
    processTIMinthom,      // 3
    processTIMintdir,      // 4
    processTIMintking,     // 5
    processUSBval,         // 6
    processEXTintWaterIn,  // 7
    processEXTintWaterOut, // 8
    processinternalH2P,    // 9
    processinternalPh2H,   // A
    processinternalPr2H,   // B
    processinternalW2T,    // C
    processinternalT2H,    // D
    processexternalT2S,    // E
    processexternalF2S,    // F
    processexternalD2S,    // 10
    processTIM2SP,         // 11
    processinternalPC2R    // 12
};

// statehandler table
typedef void (*fstatehandler_t)(event const *const e);
void Null(event const *const e);
void Process(event const *const e);
void Manual(event const *const e);
void Homing(event const *const e);
void SaveSettings(event const *const e);
void TransmitForce(event const *const e);

fstatehandler_t fstatehandler[] = {
    Null,
    Process,
    Manual,
    SaveSettings,
    Homing,
    TransmitForce};

/*Manual*/
void u_Null(void);
void u_TempControlON(void);
void u_TempControlOFF(void);
void u_MotorStartf(void);
void u_MotorStartb(void);
void u_MotorStop(void);
void u_ValveOpen1(void);
void u_ValveOpen2(void);
void u_ValveClose1(void);
void u_ValveClose2(void);

typedef void (*manualhandler_t)(void);
/*
uNone,
uTempControlON,
uTempControlOFF,
uMotorStartf,
uMotorStartb,
uMotorStop,
uValveOpen1,
uValveOpen2,
uValveClose1,
uValveClose2
*/

manualhandler_t manualhandler[] = {
    NULL,
    u_TempControlON,
    u_TempControlOFF,
    u_MotorStartf,
    u_MotorStartb,
    u_MotorStop,
    u_ValveOpen1,
    u_ValveOpen2,
    u_ValveClose1,
    u_ValveClose2};

/*Process*/
void p_Null(void);
void p_FillWater(void);
void p_WaterOff(void);
void p_EmptyWater(void);
void p_TempControlOn(void);
void p_TempControlOff(void);
void p_Homing(void);
void p_mStart(void);
void p_mReverse(void);
void p_mDirVal(void);
void p_Pause(void);
void p_Stop(void);
void p_Reset(void);
// void p_Repeat(void);

typedef void (*processhandler_t)(void);
/*
pNone,
pFillWater,
pWaterOff,
pEmptyWater,
pTempControlOn,
pTempControlOff,
pHoming,
pmStart,
pmDirVal,
pStop,
pPause,
pReset
*/
processhandler_t processhandler[] = {
    NULL,
    p_FillWater,
    p_WaterOff,
    p_EmptyWater,
    p_TempControlOn,
    p_TempControlOff,
    p_Homing,
    p_mStart,
    p_mReverse,
    p_mDirVal,
    p_Stop,
    p_Pause,
    p_Reset,
    // p_Repeat
};

/*Homing*/
void h_Null(void);
void h_Startpwm(void);
void h_Starttimbase(void);
void h_Stopext(void);
void h_Stoptim(void);

typedef void (*hominghandler_t)(void);
/*
hmNone,
hmStartpwm,
hmStarttimbase,
hmStopext,
hmStoptim,
*/
hominghandler_t hominghandler[] = {
    NULL,
    h_Startpwm,
    h_Stopext,
    h_Stoptim,
};

// Preious State for track
typedef struct
{
  defstate pstate;
  event pe;
} previous_t;

previous_t current;
previous_t conditional;
// USB Value conversion
void TransmitDistance(void);

// TempControl Functions
uint16_t readtemp(void);
void readforce(char *const force, int *const measuredforce);

int CalculateMotionDuration(int length_mm, int speed_mm_sec);

// exexex
uint8_t a, b, c, f, x, y, h, i, z, ab, tempxx, leng, d_usb, d_force, d_temp, temp_check, force_check;
volatile uint8_t stat = 0;
int g, d;
uint32_t m, n, e, interrupt;
int sp, m_force, p_force, r_length;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int set_length, set_speed, set_force;
void UpdateStepperSpeed(int speed_mm_sec);
void UpdateStepperLength(int length_mm, int speed_mm_sec);
int set_length = 0;
int set_interrupts = 0;
int measuredforce = 0;
int return_length, threshold;
;
uint32_t pause_distance;
uint8_t p_mode, ph_mode, cf, Process_State;
uint8_t count, status;
volatile uint8_t f_force, b_force, f_attempt;
uint8_t reverse, d_transmit, mul;
// volatile int t_distance;
int m_forcee, m_forceee, r_covered, dm_force;
int d_covered;

float dd_force, debug_force;
uint8_t abm, check_t;

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
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  HAL_Delay(500);
  HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, 1);
  HAL_Delay(1500);
  HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, 0);
  // 18/10/24
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of usbData */
  usbDataHandle = osMessageQueueNew(40, sizeof(uint8_t), &usbData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  sendtempHandle = osMessageQueueNew(1, sizeof(uint8_t), &sendtemp_attributes);
  sendforceHandle = osMessageQueueNew(1, sizeof(uint8_t), &sendforce_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of StateHandler */
  StateHandlerHandle = osThreadNew(StartStateHandler, NULL, &StateHandler_attributes);

  /* creation of Actuations */
  ActuationsHandle = osThreadNew(StartActuations, NULL, &Actuations_attributes);

  /* creation of TempControl */
  TempControlHandle = osThreadNew(StartTempControl, NULL, &TempControl_attributes);

  /* creation of transmit distance */
  trans_distanceHandle = osThreadNew(trans_distance, NULL, &trans_distance_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 65535;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1098;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 72 - 1; // Timer frequency = 10,000 Hz
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535; // Adjusted period for 1mm per interrupt
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Heater_Switch_GPIO_Port, Heater_Switch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | Valve1_Pin | Valve2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Heater_Pin | Pump_Out_Pin | Pump_In_Pin | LED1_Pin | LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, En_Pin | Dir_Pin | NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Water_Out_Pin */
  GPIO_InitStruct.Pin = Water_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Water_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Water_In_Pin */
  GPIO_InitStruct.Pin = Water_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Water_In_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Min_Lim_Pin Max_Lim_Pin */
  GPIO_InitStruct.Pin = Min_Lim_Pin | Max_Lim_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Heater_Switch_Pin */
  GPIO_InitStruct.Pin = Heater_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Heater_Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 Valve1_Pin
                           Valve2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | Valve1_Pin | Valve2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Heater_Pin Pump_Out_Pin Pump_In_Pin LED1_Pin
                           LED2_Pin */
  GPIO_InitStruct.Pin = Heater_Pin | Pump_Out_Pin | Pump_In_Pin | LED1_Pin | LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : En_Pin Dir_Pin NSS_Pin */
  GPIO_InitStruct.Pin = En_Pin | Dir_Pin | NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Air_Valve_Pin */
  GPIO_InitStruct.Pin = Air_Valve_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Air_Valve_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  count++;
  interrupt++;
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  if (GPIO_Pin == Max_Lim_Pin)
  {
    GPIO_PinState pinState = HAL_GPIO_ReadPin(Dir_GPIO_Port, Dir_Pin);
    status = 1;

    // Check if the pin is already low
    if (pinState == GPIO_PIN_RESET)
    {
      stat = 2;
      osThreadFlagsSet(StateHandlerHandle, 0x00000002);
      HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, 1);
    }
  }
  else if (GPIO_Pin == Min_Lim_Pin)
  {
    status = 2;
    GPIO_PinState pinState = HAL_GPIO_ReadPin(Dir_GPIO_Port, Dir_Pin);

    // Check if the pin is already high
    if (pinState == GPIO_PIN_SET)
    {
      HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, 0);
      stat = 1;
      osThreadFlagsSet(StateHandlerHandle, 0x00000001);
    }
  }
}

// State Init
void stateInit(defstate *const state, event *const e)
{
  *state = sHoming;
  e->xhoming = hmStartpwm;
}

// State Machine
void statemachine(defstate *const state, event *const e)
{
  uint32_t flags = 0;
  flags = osThreadFlagsWait(0x7FFF, osFlagsWaitAny, 0);
  if (flags >= 0x00000001 && flags < 0x00080000) // Replace MAX_STATEFINDER_INDEX with the actual max index of statefinder
  {
    a = flags;
    statefinder[flags](state, e);
    savecurrentstate(state, e);
  }
  // }
  osThreadFlagsClear(flags);
  saveconditionalstate(state, e);

  fstatehandler[*state](e);

  resetstatemachine(state, e);
}

// Reset State
void resetstatemachine(defstate *const state, event *const e)
{
  *state = NULL;
  e->xprocess = pNone;
  e->xhoming = hmNone;
  e->xmanual = uNone;
}
// Save previous state
void savecurrentstate(defstate const *const state, event const *const e)
{
  current.pstate = *state;
  current.pe = *e;
}

void saveconditionalstate(defstate const *const state, event const *const e)
{
  if (*state == sProcess)
  {
    conditional.pe = *e;
    conditional.pstate = *state;
  }
  else if (*state == sManual)
  { //{NULL, pNone, uNone, amNone, hmNone};
    conditional.pe.xhoming = hmNone;
    conditional.pe.xprocess = pNone;
    conditional.pe.xmanual = uNone;
    conditional.pstate = NULL;
  }
}
// covert values recieved from usb to usable number
void convertvalues(void) // only distance and speed
{
}
/*State Finder from EXT,USB,TIM*/
void processUSBval(defstate *const state, event *const e)
{
  char task;
  char ev;
  int permission, process;
  char operation[10];
  char direction[10];
  volatile int speed, dist, temp, force;
  uint8_t xtemp, xforce, xspeed, xdist;

  if (buff[1] == '1' || buff[1] == '2')
  {
    sscanf((char *)buff, "*%c:%c:%d:%d:%d:%d#", &task, &ev, &speed, &dist, &temp, &force);
    xtemp = temp;
    xforce = force;
    set_force = force;
    set_length = dist;
    set_speed = speed;

    // Put target temperature and force values in message queues
    osMessageQueuePut(sendtempHandle, &xtemp, NULL, 0);
    osMessageQueuePut(sendforceHandle, &xforce, NULL, 0);
  }

  /*=================PROCESS========================*/

  if (buff[1] == '3' && buff[3] == '5')
  {
    // osMessageQueuePut(sendtempHandle, &xtemp, NULL, 0);
  }
  if (buff[1] == '1' && buff[3] == '1')
  {
    int temp = readtemp(); // Read the current temperature
    if (temp < xtemp)
    {
      ab++;
      // Send heating notification
      taskENTER_CRITICAL();
      CDC_Transmit_HS((uint8_t *)hfeedback, strlen(hfeedback));
      taskEXIT_CRITICAL();

      while (temp < xtemp)
      {
        HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_SET);
        osDelay(1000);
        temp = readtemp();
      }
      HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
    }

    taskENTER_CRITICAL();
    CDC_Transmit_HS((uint8_t *)sfeedback, strlen(sfeedback));
    taskEXIT_CRITICAL();
    p_mode = 1;
    ph_mode = 1;
    *state = sProcess;
    e->xprocess = pmStart;
  }
  else if (buff[1] == '1' && buff[3] == '3')
  {
    *state = sProcess;
    e->xprocess = pReset;
  }
  else if (buff[1] == '1' && buff[3] == '2')
  {
    *state = sProcess;
    e->xprocess = pPause;
    taskENTER_CRITICAL();
    CDC_Transmit_HS((uint8_t *)pfeedback, strlen(pfeedback)); // feedback
    taskEXIT_CRITICAL();
  }

  /*=================MANUAL========================*/

  else if (buff[1] == '2' && buff[3] == '4' && buff[5] == '1' && buff[7] == '1') // 2411
  {
    *state = sManual;
    e->xmanual = uMotorStartf;
  }
  else if (buff[1] == '2' && buff[3] == '4' && buff[5] == '2' && buff[7] == '1') // 2421
  {
    *state = sManual;
    e->xmanual = uMotorStop;
  }
  else if (buff[1] == '2' && buff[3] == '4' && buff[5] == '1' && buff[7] == '2') // 2412
  {
    *state = sManual;
    e->xmanual = uMotorStartb;
  }
  else if (buff[1] == '2' && buff[3] == '4' && buff[5] == '2' && buff[7] == '2') // 2422
  {
    *state = sManual;
    e->xmanual = uMotorStop;
  }
  else if (buff[1] == '2' && buff[3] == '5' && buff[5] == '1') // 251
  {
    *state = sManual;
    e->xmanual = uValveOpen1;
  }
  else if (buff[1] == '2' && buff[3] == '5' && buff[5] == '2') // 252
  {
    *state = sManual;
    e->xmanual = uValveClose1;
  }
  else if (buff[1] == '2' && buff[3] == '6' && buff[5] == '1') // 261
  {
    *state = sManual;
    e->xmanual = uValveOpen2;
  }
  else if (buff[1] == '2' && buff[3] == '6' && buff[5] == '2') // 262
  {
    *state = sManual;
    e->xmanual = uValveClose2;
  }
  else if (buff[1] == '2' && buff[3] == '7' && buff[5] == '1') // 271
  {
    *state = sManual;
    e->xmanual = uTempControlON;
  }
  else if (buff[1] == '2' && buff[3] == '7' && buff[5] == '2') // 272
  {
    *state = sManual;
    e->xmanual = uTempControlOFF;
  }

  /*=================MANUAL-FEEDBACK========================*/
  if (*state == sManual)
  {
    if (e->xmanual != 0)
    {
      taskENTER_CRITICAL();
      CDC_Transmit_HS((uint8_t *)mfeedback, strlen(mfeedback));
      taskEXIT_CRITICAL();
    }
  }
}

void processEXTinthommin(defstate *const state, event *const e)
{
  *state = sHoming;
  e->xhoming = hmStopext;
}
void processEXTinthommax(defstate *const state, event *const e)
{
  *state = sHoming;
  e->xhoming = hmStopext;
}
void processTIMinthom(defstate *const state, event *const e)
{
  *state = sHoming;
  e->xhoming = hmStoptim;
}
void processTIMintdir(defstate *const state, event *const e)
{
  *state = sProcess;
  e->xprocess = pmDirVal;
}
void processTIMintking(defstate *const state, event *const e)
{
  *state = sProcess;
  e->xprocess = pStop;
}
void processEXTintWaterIn(defstate *const state, event *const e)
{

  *state = sProcess;
  e->xprocess = pWaterOff;
}
void processEXTintWaterOut(defstate *const state, event *const e)
{
  *state = sProcess;
  e->xprocess = pWaterOff;
}
void processinternalH2P(defstate *const state, event *const e)
{
  *state = sProcess;
  e->xprocess = pmStart;
}
void processinternalPh2H(defstate *const state, event *const e)
{
  *state = sHoming;
  e->xhoming = hmStartpwm;
}
void processinternalPr2H(defstate *const state, event *const e)
{
  *state = sHoming;
  e->xhoming = hmStartpwm;
}
void processinternalW2T(defstate *const state, event *const e)
{
  *state = sProcess;
  e->xprocess = pTempControlOn;
}
void processinternalT2H(defstate *const state, event *const e)
{
  *state = sHoming;
  e->xhoming = hmStartpwm;
}
void processexternalT2S(defstate *const state, event *const e)
{
  *state = sProcess;
  e->xprocess = pTempControlOff;
}
void processexternalF2S(defstate *const state, event *const e) // if Force is reached
{
  *state = sProcess;
  e->xprocess = pStop;
}
void processexternalD2S(defstate *const state, event *const e) // if Distance is reached
{
  *state = sProcess;
  e->xprocess = pStop;
}
void processTIM2SP(defstate *const state, event *const e)
{
  stat = 0x11;
  *state = sProcess;
  e->xprocess = pmStart;
}
void processinternalPC2R(defstate *const state, event *const e)
{
  stat = 0x12;
  *state = sProcess;
  e->xprocess = pmReverse;
}
/*State Handlers*/
void Null(event const *const e)
{
}
void Process(event const *const e)
{
  processhandler[e->xprocess]();
}
void Manual(event const *const e)
{
  manualhandler[e->xmanual]();
}
void Homing(event const *const e)
{
  hominghandler[e->xhoming]();
}
void SaveSettings(event const *const e)
{
}
void TransmitForce(event const *const e)
{
}
void TransmitDistance(void)
{
  char dist[25];
  static int prev_d_transmit = -1; // Track previous mode

  // Reset counters on mode change
  if (prev_d_transmit != d_transmit)
  {
    Interrupt_Tim5 = 0;
    f_attempt = 0;
    prev_d_transmit = d_transmit;
  }

  if (d_transmit == 1)
  {
    if (f_attempt == 0)
    {
      d_covered = (Interrupt_Tim5 / 1000) + 1;
    }
    else if (f_attempt % 2 == 0)
    {
      int mul = (f_attempt / 2) * 10;
      // Prevent underflow
      int timer_val = Interrupt_Tim5 / 1000;
      d_covered = (timer_val > mul) ? (timer_val - mul + 1) : 1;
    }
    snprintf(dist, sizeof(dist), "*DIS:%d#", d_covered);
  }
  else if (d_transmit == 0)
  {
    int calculated;
    if (f_attempt == 0)
    {
      calculated = (Interrupt_Tim5 / 1000);
      r_covered = d_covered - calculated;
    }
    else if (f_attempt % 2 == 0)
    {
      int mul = (f_attempt / 2) * 10;
      int timer_val = Interrupt_Tim5 / 1000;
      calculated = (timer_val > mul) ? (timer_val - mul) : 0;
      r_covered = d_covered - calculated;
    }
    snprintf(dist, sizeof(dist), "*DIS:%d#", (r_covered > 0) ? r_covered - 1 : 0);
  }

  taskENTER_CRITICAL();
  CDC_Transmit_HS((uint8_t *)dist, strlen(dist));
  taskEXIT_CRITICAL();
}
/* Event Handlers*/
void u_TempControlON(void)
{
  HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, 1); // Turn off heater
}
void u_TempControlOFF(void)
{
  HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, 0); // Turn off heater
}
void u_MotorStartf(void)
{
  HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, 1);
  HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
  __HAL_TIM_SetCounter(&htim1, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim1, 600 - 1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}
void u_MotorStartb(void)
{
  HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, 0);
  HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
  __HAL_TIM_SetCounter(&htim1, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim1, 600 - 1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}
void u_MotorStop(void)
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}
void u_ValveOpen1(void)
{
  HAL_GPIO_WritePin(Air_Valve_GPIO_Port, Air_Valve_Pin, 0);
}
void u_ValveOpen2(void)
{
  HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, 1);
}
void u_ValveClose1(void)
{
  HAL_GPIO_WritePin(Air_Valve_GPIO_Port, Air_Valve_Pin, 1);
}
void u_ValveClose2(void)
{
  HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, 0);
}

/*Process*/
void p_FillWater(void)
{
}
void p_WaterOff(void)
{
}
void p_EmptyWater(void)
{
}
void p_TempControlOn(void)
{

  HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, 1);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
  osThreadFlagsSet(TempControlHandle, 0x01);
}
void p_TempControlOff(void)
{
  HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, 0);
  stat = 0x11;
  osThreadFlagsSet(StateHandlerHandle, 0x00000011);
}
void p_Homing(void)
{
  // 10
  stat = 10;
  osThreadFlagsSet(StateHandlerHandle, 0x0000000A); // same as homing (implemet a method to call homing)
}

void p_Pause(void)
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_Base_Stop_IT(&htim4);
  HAL_TIM_Base_Stop_IT(&htim5);
  HAL_GPIO_WritePin(Pump_In_GPIO_Port, Pump_In_Pin, 0);
  HAL_GPIO_WritePin(Pump_Out_GPIO_Port, Pump_Out_Pin, 0);
}
void p_Stop(void)
{

  Process_State++;

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_Base_Stop_IT(&htim4);
  HAL_TIM_Base_Stop_IT(&htim5);
  Interrupt_Tim1 = 0;
  Interrupt_Tim5 = 0;
  HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, 0); // Turn off heater
  HAL_GPIO_WritePin(Air_Valve_GPIO_Port, Air_Valve_Pin, 0);
  osDelay(1500);
  if (Process_State == 1 && d_force == 0)
  {
    d_usb = 1;
    osThreadFlagsSet(StateHandlerHandle, 0x00000012);
  }
  else
  {
    d_usb = 2;
    p_mode = 0;
    ph_mode = 0;
    osThreadFlagsSet(StateHandlerHandle, 0x000000A);
  }
}
void p_Reset(void)
{
  // 11
  stat = 11;
  HAL_GPIO_WritePin(Air_Valve_GPIO_Port, Air_Valve_Pin, 0);
  osThreadFlagsSet(StateHandlerHandle, 0x0000000B); // same as homing (implemet a method to call homing)
  osThreadFlagsSet(ActuationsHandle, 0x00000001);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_Base_Stop_IT(&htim5);
  p_mode = 0;
  ph_mode = 0;
  Interrupt_Tim1 = 0;
  Interrupt_Tim5 = 0;
}

void p_mStart(void)
{
  osThreadFlagsSet(TempControlHandle, 0x01);
  if (conditional.pe.xprocess == pmStart) // conditions are for stop pause start process...
  {
    HAL_GPIO_WritePin(Air_Valve_GPIO_Port, Air_Valve_Pin, 0);
    HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, 0);
    HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
    __HAL_TIM_SetCounter(&htim1, 0);
    UpdateStepperSpeed(set_speed);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    Process_State = 0;
    reverse = 0;
    d_transmit = 1;
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim5);
  }
  else if (conditional.pe.xprocess == pNone)
  {
    HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
    __HAL_TIM_SetCounter(&htim1, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim1, 1000 - 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim5);
  }
}
void p_mReverse(void)
{
  HAL_GPIO_WritePin(Air_Valve_GPIO_Port, Air_Valve_Pin, 0);
  HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, 1);
  HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
  f_attempt = 0;
  d_transmit = 0;
  __HAL_TIM_SetCounter(&htim1, 0);
  UpdateStepperSpeed(set_speed);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim5);
}
void p_mDirVal(void)
{
  Interrupt_Tim1 = 0;
  f_attempt++;
  HAL_GPIO_TogglePin(Dir_GPIO_Port, Dir_Pin);
  HAL_GPIO_TogglePin(Air_Valve_GPIO_Port, Air_Valve_Pin);
}

/*Homing*/
void h_Startpwm(void)
{
  taskENTER_CRITICAL();
  CDC_Transmit_HS((uint8_t *)rfeedback, strlen(rfeedback));
  taskEXIT_CRITICAL();
  force_check = 0;
  temp_check = 0;
  f_attempt = 0;
  d_covered = 0;
  r_covered = 0;
  HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, 0);
  HAL_GPIO_WritePin(Air_Valve_GPIO_Port, Air_Valve_Pin, 1);
  HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, 1);
  HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
  d_force = 0;
  __HAL_TIM_SetCounter(&htim1, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim1, 500 - 1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void h_Stopext(void)
{
  Interrupt_Tim1 = 0;
  HAL_TIM_Base_Start_IT(&htim1);
}

void h_Stoptim(void)
{
  HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 0);
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  Interrupt_Tim1 = 0;
  Interrupt_Tim5 = 0;
  taskENTER_CRITICAL();
  CDC_Transmit_HS((uint8_t *)nfeedback, strlen(nfeedback));
  taskEXIT_CRITICAL();

  if (conditional.pe.xprocess == pHoming)
  {
    // 9
    stat = 9;
    osThreadFlagsSet(StateHandlerHandle, 0x00000009); // to read and learn from this this..(this to is notify that homing was done during process)
  }
}

void UpdateStepperSpeed(int speed_mm_sec)
{

  const int STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / LEAD_SCREW_PITCH;
  int steps_per_sec = STEPS_PER_MM * speed_mm_sec; // For 1mm/sec, this will be STEPS_PER_MM
  uint32_t timer_period = (TIMER_CLOCK_FREQ / steps_per_sec) - 1;
  if (timer_period > 0xFFFF)
  {
    timer_period = 0xFFFF; // Limit the period to 16-bit max value
  }

  __HAL_TIM_SET_AUTORELOAD(&htim1, timer_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, timer_period / 2);

  __HAL_TIM_SET_AUTORELOAD(&htim5, timer_period);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, timer_period / 2); // 50% duty cycle
}

/*Temp Read*/
uint16_t readtemp(void)
{
  uint8_t rxData[2]; // Buffer to receive 16 bits of data
  uint16_t tempData;

  // Assert CS low to select MAX6675
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 0);

  // Send dummy byte to read temperature data
  HAL_SPI_Receive(&hspi3, rxData, 2, HAL_MAX_DELAY);

  // Deassert CS high to deselect MAX6675
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 1);

  // Combine received bytes into 16-bit temperature data
  tempData = (rxData[0] << 8) | rxData[1];

  // Extract temperature value (12-bit data)
  tempData = (tempData >> 3); // Convert to Celsius
  return tempData * 0.25;
  // Return the temperature value
}

/*Force Read*/
void readforce(char *const force, int *const measuredforce)
{
  uint16_t readings[100];
  float sum = 0.0;
  float measured_force = 0;

  for (uint8_t i = 0; i < 100; i++)
  {
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, portMAX_DELAY);
    readings[i] = HAL_ADC_GetValue(&hadc2);
    sum += readings[i];
  }

  float average = sum / 100;
  dd_force = average;
  measured_force = average * (5 / 4095.0) * 718.84628047;

  measured_force = (2120 - 807.05) - measured_force;
  debug_force = measured_force;
  if (measured_force <= -80 || measured_force >= 20)
  {
    g = measured_force > 0 ? measured_force - 20 : measured_force + 70;
  }
  else
  {
    g = 0;
  }

  *measuredforce = measured_force * 1000; // Convert to appropriate units (force in mN)
  m = *measuredforce;
  snprintf(force, 16, "*FRC:%d#", g);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartStateHandler */
/**
  * @brief  Function implementing the StateHandler thread.
  * @param  argument: Not used


  * @retval None
  */
/* USER CODE END Header_StartStateHandler */
void StartStateHandler(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  defstate state = NULL;
  event e = {NULL, pNone, uNone, hmNone}; // Initialize all to None

  stateInit(&state, &e);
  buff[1] = '2';
  /* Infinite loop */
  for (;;)
  {

    statemachine(&state, &e);

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartActuations */
/**
 * @brief Function implementing the Actuations thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartActuations */
void StartActuations(void *argument)
{
  /* USER CODE BEGIN StartActuations */
  char force[15];

  uint8_t standardforce = 1;
  int flags = 0;
  uint8_t forceFlag = 0;
  uint8_t returningFlag = 0;

  /* Infinite loop */
  for (;;)
  {
    if (standardforce == 1)
    {
      b = 1;
      osMessageQueueGet(sendforceHandle, &standardforce, NULL, portMAX_DELAY);
    }

    flags = osThreadFlagsWait(0x00000001, osFlagsWaitAny, 0);
    d = standardforce;

    if (flags != 0x00000001)
    {
      readforce(force, &measuredforce);
      taskENTER_CRITICAL();
      CDC_Transmit_HS((uint8_t *)force, strlen(force)); // Send the force value over USB
      taskEXIT_CRITICAL();
      m_forcee = measuredforce;
      measuredforce = measuredforce / 100000;
      m_forceee = measuredforce;
      if (ph_mode == 1)
      {
        if (measuredforce >= set_force) // Check if measured force meets or exceeds the standard force
        {
          force_check = 2;
          d_force++;
          osThreadFlagsSet(StateHandlerHandle, 0x00000005);
        }
        else
        {
          force_check = 1;
        }
      }
      else
      {
        forceFlag = 0;
        // m_force = 1;
      }
      //}

      osDelay(100); // Small delay before next reading
    }
  }
  /* USER CODE END StartActuations */
}

/* USER CODE BEGIN Header_StartTempControl */
/**
 * @brief Function implementing the TempControl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTempControl */
void StartTempControl(void *argument)
{
  int temperature;
  char temp[15];
  uint8_t standardtemp = 5;
  const int HYSTERESIS = 2; // Degrees Celsius hysteresis to prevent rapid cycling

  for (;;)
  {
    osMessageQueueGet(sendtempHandle, &standardtemp, NULL, osWaitForever);

    while (1)
    {
      temperature = readtemp();
      snprintf(temp, sizeof(temp), "*TEP:%d#", temperature);

      taskENTER_CRITICAL();
      CDC_Transmit_HS((uint8_t *)temp, strlen(temp));
      taskEXIT_CRITICAL();

      if (p_mode == 1)
      {
        // Turn heater on if below (setpoint - hysteresis), off if above (setpoint + hysteresis)
        if (temperature < standardtemp)
        {
          HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_SET);
        }
        else if (temperature >= standardtemp)
        {
          HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
        }
      }

      // Check for updated setpoint non-blocking
      osMessageQueueGet(sendtempHandle, &standardtemp, NULL, 0);

      osDelay(500); // Reduce delay to 500ms for faster response
    }
  }
}
void trans_distance(void *argument)
{
  while (1)
  {
    check_t++;
    TransmitDistance();
    osDelay(100);
  }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    Interrupt_Tim1++;
    if (current.pstate != sProcess) // global variable to get condition
    {
      if (Interrupt_Tim1 == 15000)
      {
        // 3
        stat = 3;
        osThreadFlagsSet(StateHandlerHandle, 0x00000003);
      }
    }
    else if (current.pstate == sProcess)
    {

      if (Interrupt_Tim1 == 10000) // for changing motor direction during process
      {
        stat = 4;
        osThreadFlagsSet(StateHandlerHandle, 0x00000004);
      }
    }
    else if (Process_State == 1)
    {
      int remainder = threshold % 10000;
      remainder = (remainder == 0) ? 10000 : remainder;

      int target;
      if (Interrupt_Tim1 == 0)
      {
        target = remainder; 
      }
      else
      {
        target = 10000; 
      }

      if (Interrupt_Tim1 == target)
      {
        stat = 4;
        osThreadFlagsSet(StateHandlerHandle, 0x00000004);
        Interrupt_Tim1 = 0;
      }
    }
  }

  else if (htim->Instance == TIM5)
  {
    Interrupt_Tim5++;

    int last_digit = set_length % 10;
    if (last_digit == 0)
    {
      threshold = (set_length * 2000) - 10000;
    }
    else
    {
      threshold = (set_length * 2000) - (last_digit * 1000);
    }

    if (Interrupt_Tim5 >= threshold)
    {
      stat = 5;
      osThreadFlagsSet(StateHandlerHandle, 0x00000005);
    }
  }
}
/* USER CODE END Callback 1 */

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

#ifdef USE_FULL_ASSERT
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