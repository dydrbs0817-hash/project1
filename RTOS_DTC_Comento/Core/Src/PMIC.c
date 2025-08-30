//* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "PMIC.h"
#include "DTC.h"
#include "EEPROM.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// SPI 칩셀렉트 핀
#define SPI1_CS_GPIO_Port  GPIOB
#define SPI1_CS_Pin        GPIO_PIN_0	// 0번PB0를 SPI1의 CS로 이용
#define SPI1_CS_HIGH()     HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)
#define SPI1_CS_LOW()      HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
// PMIC I2C 레지스터 주소(enum)
// TODO: 데이터시트 표의 실제 주소로 교체 해야함


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
UART_HandleTypeDef huart4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2CTask */
osThreadId_t I2CTaskHandle;
const osThreadAttr_t I2CTask_attributes = {
  .name = "I2CTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SPITask */
osThreadId_t SPITaskHandle;
const osThreadAttr_t SPITask_attributes = {
  .name = "SPITask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANTask */
osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CANTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanQueue */
osMessageQueueId_t CanQueueHandle;
const osMessageQueueAttr_t CanQueue_attributes = {
  .name = "CanQueue"
};
/* Definitions for CommMutexHandle */
osMutexId_t CommMutexHandleHandle;
const osMutexAttr_t CommMutexHandle_attributes = {
  .name = "CommMutexHandle"
};


/* USER CODE BEGIN PV */
// PMIC I2C DMA 상태머신
typedef enum {
  PMIC_DMA_IDLE = 0,
  PMIC_DMA_READING_VOLT,
  PMIC_DMA_READING_CURR,
  PMIC_DMA_READING_TEMP
} PMIC_DMA_State_t;

// I2C DMA 상태머신의 현재 단계를 저장
static volatile PMIC_DMA_State_t g_pmic_state = PMIC_DMA_IDLE;
// 전압 Fault 레지스터에서 읽어온 8바이트
static PMIC_Fault8_t g_fault_volt = {0};
// 전류(Current) Fault 레지스터에서 읽어온 8바이트
static PMIC_Fault8_t g_fault_curr = {0};


// 루프 단계: I2C -> SPI -> CAN -> UART
typedef enum { PH_I2C=0, PH_SPI, PH_CAN, PH_UART } loop_phase_t;
static loop_phase_t s_phase = PH_I2C;

#define DTC_CODE_BRAKE_PRESSURE_UV   (0xC121)  // DTC 예시코드
#define DTC_CODE_BRAKE_MOTOR_OC      (0xC122)  // DTC 예시코드
static DTC_Record_t g_last_dtc = {0};          // 마지막 발생 DTC


// CAN 수신 버퍼 (예시)
static CAN_RxHeaderTypeDef g_can_rx_hdr;
static uint8_t             g_can_rx_data[8];

// 주기 로그 타이밍
static uint32_t g_last_log_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);

void StartDefaultTask(void *argument);
void StartI2CTask(void *argument);
void StartSPITask(void *argument);
void StartCANTask(void *argument);
void StartUARTTask(void *argument);


/* USER CODE BEGIN PFP */
// ===== UART 로깅 함수 =====
// printf 처럼 문자열을 만들어서 UART4로 전송하는 함수
// → 디버깅 메시지, Fault 상태 로그 출력
void UART4_Log(const char *fmt, ...);

// ===== CAN 필터 설정 및 시작 함수 =====
// CAN1을 "모든 메시지를 수신"하도록 필터를 설정하고 CAN을 시작
// → 복잡한 필터링 설정 대신 간단히 통신이 되는지 확인할 때 사용
static void CAN1_ConfigSimple(void);

// ===== CAN 데이터 전송 함수 =====
// 8바이트 데이터를 표준 ID(0x123)로 CAN 버스로 송신
// → I2C에서 읽은 Fault 상태 데이터를 그대로 CAN에 송신할 때 사용
static void CAN1_Send8(const uint8_t *data);

/* ADC helper */
static uint16_t ADC_Read_mV(ADC_HandleTypeDef *hadc, uint32_t channel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */
// I2C에서 8바이트 읽기(메모리 읽기)
// PMIC 내부 레지스터인 reg에서 연속 8바이트를 폴링 방식으로 I2C로 읽어 out(PMIC_Fault_8_t)를 가리키는 포인터 -> raw에 채움
// out->raw : out이 가리키는 PMIC_Fault8_t의 raw 배열 (= frame.raw)

void PMIC_ConfigPGasInput(void)
{
  HAL_GPIO_DeInit(PMIC_PG_GPIO_Port, PMIC_PG_Pin);

  GPIO_InitTypeDef gi = {0};
  gi.Pin   = PMIC_PG_Pin;
  gi.Mode  = GPIO_MODE_INPUT;   // 입력으로
  gi.Pull  = GPIO_PULLUP;       // 회로에 따라 PULLDOWN/NOPULL로 조정
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PMIC_PG_GPIO_Port, &gi);
}


// PG 안정 확인
// timeout_ms 동안, 'stable_count' 회 연속 원하는 레벨(Active-High/Low)인가?
static bool PMIC_WaitForPG(GPIO_TypeDef *port, uint16_t pin,
                           uint32_t timeout_ms, uint8_t stable_count)
{
  uint32_t t0 = HAL_GetTick();
  uint8_t good = 0;

  while ((HAL_GetTick() - t0) < timeout_ms) {
    GPIO_PinState s = HAL_GPIO_ReadPin(port, pin);
    bool ok = PMIC_PG_ACTIVE_HIGH ? (s == GPIO_PIN_SET) : (s == GPIO_PIN_RESET);

    if (ok) {
      if (++good >= stable_count) return true;  // 충분히 안정
    } else {
      good = 0; // 한 번이라도 틀리면 연속 카운터 초기화
    }

    HAL_Delay(1); // 1ms 주기로 샘플링
  }
  return false; // 타임아웃
}

// I2C 기본값 체크
static bool PMIC_CheckI2CDefaults(void)
{
  // 슬레이브 응답 확인
  if (HAL_I2C_IsDeviceReady(&hi2c1, PMIC_I2C_ADDR, 3, 10) != HAL_OK) {
    UART4_Log("[PMIC] I2C not ready\r\n");
    return false;
  }

  // 1) Fault: 전압/전류/온도 → 파워온 직후 보통 '모두 0' 기대
  PMIC_Fault8_t v = {0}, c = {0};

  if (PMIC_Read8(&hi2c1, PMIC_REG_FAULT_VOLT, v.raw) != HAL_OK) {
    UART4_Log("[PMIC] read VOLT fault fail\r\n"); return false;
  }
  if (PMIC_Read8(&hi2c1, PMIC_REG_FAULT_CURR, c.raw) != HAL_OK) {
    UART4_Log("[PMIC] read CURR fault fail\r\n"); return false;
  }

  // 2) '모두 0'인지 검사 (한 바이트라도 0이 아니면 초기 고장 래치로 판단)
  bool volt_zero = true, curr_zero = true;
  for (int i = 0; i < 8; ++i) {
    if (v.raw[i] != 0x00) volt_zero = false;
    if (c.raw[i] != 0x00) curr_zero = false;
  }

  if (!volt_zero) {
    UART4_Log("[PMIC] FAULT_VOLT not zero @POR (check DS/board)\r\n");
    return false;
  }
  if (!curr_zero) {
    UART4_Log("[PMIC] FAULT_CURR not zero @POR (check DS/board)\r\n");
    return false;
  }

  UART4_Log("[PMIC] Defaults OK: FAULT(V/C/T)=all-zero\r\n");

  return true; // 모든 기본값 검사 통과
}



// CAN 필터 간단 설정 후 시작
static void CAN1_ConfigSimple(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterActivation = ENABLE;
  f.FilterBank = 0;
  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  f.FilterMode  = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan1, &f);
  HAL_CAN_Start(&hcan1);

  //수신 알림 활성화 + NVIC
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

// CAN 8바이트 송신
static void CAN1_Send8(const uint8_t *data)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint32_t mbox;
  tx.IDE = CAN_ID_STD;
  tx.StdId = 0x123;  // TODO: 시스템에 맞게 data 변경
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = 8;
  HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)data, &mbox);

  uint32_t t0 = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(&hcan1, mbox)) {
    if (HAL_GetTick() - t0 > 10) break;
  }
}


// UART 텍스트 출력
static void UART4_Log(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf)) n = sizeof(buf);
  HAL_UART_Transmit(&huart4, (uint8_t*)buf, (uint16_t)n, 100);
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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  // DTC(=EEPRO) 초기화 (SPI1, CS=PB0)
  if (!DTC_Init(&hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin)) {
    UART4_Log("[DTC] init failed\r\n");
    Error_Handler();
  }

  // CAN 필터 설정 및 시작, interrupt 활성화
  CAN1_ConfigSimple();

  //PG 핀을 입력으로 재설정, 전원 정상(PG=OK) 안정될 때까지 잠시 대기 -> 연속 5회
  PMIC_ConfigPGasInput();
  if (!PMIC_WaitForPG(PMIC_PG_GPIO_Port, PMIC_PG_Pin, 200, 5)) {
    UART4_Log("[PMIC] PG not stable. abort init\r\n");
    Error_Handler();
  }

  // I2C 기본값(디폴트) 확인 (Fault 블록들이 Power-On에서 all-zero인지)
  if (!PMIC_CheckI2CDefaults()) {
    UART4_Log("[PMIC] I2C defaults check failed\r\n");
    Error_Handler();
  }

  // MCU/IC 전압 인가(I2C Write) : BUCK1을 원하는 전압으로 설정 후 Enable
  if (PMIC_SetBuck1Voltage_mV(&hi2c1, 1200) != HAL_OK) {
    UART4_Log("[PMIC] BUCK1 apply failed\r\n");
    Error_Handler();
  }
  // 전압 안정화 짧은 대기 + PG 재확인
  HAL_Delay(2);

  // I2C DMA 감시 시작: 전압 Fault 레지스터 8바이트 읽기
  g_pmic_state = PMIC_DMA_READING_VOLT;
  if (HAL_I2C_Mem_Read_DMA(&hi2c1,
                             PMIC_I2C_ADDR,                     // 7bit 주소<<1
                             (uint16_t)PMIC_REG_FAULT_VOLT,     // 시작 레지스터
                             I2C_MEMADD_SIZE_8BIT,              // 내부주소 8bit
                             g_fault_volt.raw,                  // 수신 버퍼(8바이트)
                             8) != HAL_OK) {
      UART4_Log("[PMIC] I2C DMA start fail\r\n");
      Error_Handler();
    }

  UART4_Log("[PMIC] init OK. entering main loop...\r\n");


  /* USER CODE END 2 */

  /* Init scheduler */
  //osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of CommMutexHandle */
  //CommMutexHandleHandle = osMutexNew(&CommMutexHandle_attributes);

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
  /* creation of CanQueue */
  //CanQueueHandle = osMessageQueueNew (8, 8, &CanQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of I2CTask */
  //I2CTaskHandle = osThreadNew(StartI2CTask, NULL, &I2CTask_attributes);

  /* creation of SPITask */
  //SPITaskHandle = osThreadNew(StartSPITask, NULL, &SPITask_attributes);

  /* creation of CANTask */
  //CANTaskHandle = osThreadNew(StartCANTask, NULL, &CANTask_attributes);

  /* creation of UARTTask */
  //UARTTaskHandle = osThreadNew(StartUARTTask, NULL, &UARTTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  switch (s_phase) {

	     case PH_I2C:
	         // I2C 단계: PMIC Fault 스냅샷 읽기 시작 (DMA)
	         if (g_pmic_state == PMIC_DMA_IDLE) {
	             g_pmic_state = PMIC_DMA_READING_VOLT;
	             (void)HAL_I2C_Mem_Read_DMA(&hi2c1,
	                                        PMIC_I2C_ADDR,
	                                        (uint16_t)PMIC_REG_FAULT_VOLT,
	                                        I2C_MEMADD_SIZE_8BIT,
	                                        g_fault_volt.raw, 8);
	         }
	         s_phase = PH_SPI;
	         break;

	     case PH_SPI: {
	         // SPI 단계: EEPROM에서 최신 DTC 1건을 읽어 확인 (내부적으로 SPI DMA 사용)
	    	 DTC_Record_t rec;
	    	 (void)DTC_ReadLatestRecord(&rec);
	         s_phase = PH_CAN;
	         break;
	     }

	     case PH_CAN:
	         // CAN 단계: 최신 DTC를 표준 ID(0x700)로 1프레임 송신
	         (void)DTC_SendLatestOverCAN(&hcan1, 0x700);
	         s_phase = PH_UART;
	         break;

	     case PH_UART: {
	         static uint32_t last_log = 0;
	         uint32_t now = HAL_GetTick();
	         if (now - last_log >= 100u) {
	             last_log = now;
	             UART4_Log("[OK] alive, UV:%u OC:%u OT:%u\r\n",
	               (unsigned)(g_fault_volt.bits.buck1_uv | g_fault_volt.bits.buck2_uv),
	               (unsigned)(g_fault_curr.bits.buck1_oc | g_fault_curr.bits.buck2_oc),
	               (unsigned)(g_fault_temp.bits.temp_fault)); // ★ 온도
	         }
	         s_phase = PH_I2C;
	         break;
      }

      } // switch
  }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;         // 입력
  GPIO_InitStruct.Pull = GPIO_PULLUP;             // 보드에 맞춰 PULLUP/PULLDOWN 조정
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN CALLBACKS */

extern void EEPROM_OnTxCpltIRQ(SPI_HandleTypeDef *hspi);
extern void EEPROM_OnRxCpltIRQ(SPI_HandleTypeDef *hspi);

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    EEPROM_OnTxCpltIRQ(hspi);  // CS HIGH + 상태 IDLE
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    EEPROM_OnRxCpltIRQ(hspi);  // CS HIGH + 상태 IDLE
}


// I2C DMA 수신 완료 콜백
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c1) return;

    if (g_pmic_state == PMIC_DMA_READING_VOLT) {
        g_pmic_state = PMIC_DMA_READING_CURR;
        HAL_I2C_Mem_Read_DMA(&hi2c1, PMIC_I2C_ADDR,
                             (uint16_t)PMIC_REG_FAULT_CURR, I2C_MEMADD_SIZE_8BIT,
                             g_fault_curr.raw, 8);
    }
    else if (g_pmic_state == PMIC_DMA_READING_CURR) {
        g_pmic_state = PMIC_DMA_READING_TEMP;
        HAL_I2C_Mem_Read_DMA(&hi2c1, PMIC_I2C_ADDR,
                             (uint16_t)PMIC_REG_FAULT_TEMP, I2C_MEMADD_SIZE_8BIT,
                             g_fault_temp.raw, 8);
    }
    else if (g_pmic_state == PMIC_DMA_READING_TEMP) {
        g_pmic_state = PMIC_DMA_IDLE;

        // 세 스냅샷으로 평가/저장 (temp 포함)
        (void)DTC_EvaluateAndSaveFromPmic(&g_fault_volt, &g_fault_curr, &g_fault_temp);

        // 다음 주기 다시 VOLT부터
        g_pmic_state = PMIC_DMA_READING_VOLT;
        HAL_I2C_Mem_Read_DMA(&hi2c1, PMIC_I2C_ADDR,
                             (uint16_t)PMIC_REG_FAULT_VOLT, I2C_MEMADD_SIZE_8BIT,
                             g_fault_volt.raw, 8);
    }
}


// CAN RX: 메시지 도착 콜백
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan != &hcan1) return;
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &g_can_rx_hdr, g_can_rx_data) == HAL_OK) {
    // 수신 처리
    UART4_Log("[CAN RX] ID:0x%03lX DLC:%d DATA:%02X %02X %02X %02X %02X %02X %02X %02X\n",
              g_can_rx_hdr.StdId, g_can_rx_hdr.DLC,
              g_can_rx_data[0], g_can_rx_data[1], g_can_rx_data[2], g_can_rx_data[3],
              g_can_rx_data[4], g_can_rx_data[5], g_can_rx_data[6], g_can_rx_data[7]);
  }
}

/* USER CODE END CALLBACKS */
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartI2CTask */
/**
* @brief Function implementing the I2CTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartI2CTask */
void StartI2CTask(void *argument)
{
  /* USER CODE BEGIN StartI2CTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartI2CTask */
}

/* USER CODE BEGIN Header_StartSPITask */
/**
* @brief Function implementing the SPITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPITask */
void StartSPITask(void *argument)
{
  /* USER CODE BEGIN StartSPITask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSPITask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void *argument)
{
  /* USER CODE BEGIN StartCANTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCANTask */
}

/* USER CODE BEGIN Header_StartUARTTask */
/**
* @brief Function implementing the UARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void *argument)
{
  /* USER CODE BEGIN StartUARTTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUARTTask */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
