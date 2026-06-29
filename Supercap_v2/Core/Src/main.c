/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum {
    SYSTEM_OFF        = 0,
    SYSTEM_CHARGING   = 1,
    SYSTEM_DISCHARGING = 2
} SystemState_t;

#pragma pack(push, 1)
typedef struct
{
    float i_Motor;
    float V_Motor;
    float i_Bat;
    float V_Cap;
    float i_Cap;
    float system_status;
    float phase;
    float control_effort;
    float clip_reason;
    float current_power;
    uint32_t tail;
} VofaData_t;
#pragma pack(pop)

float dynamic_vref;

uint16_t adc1_buffer[5] = {0U, 0U, 0U, 0U, 0U}; //holds adc values for motor current and bus voltage
// Index 0 = PC0 (IBUS)
// Index 1 = PA2 (VCAP)
// Index 2 = PA3 (ICAP)
//uint16_t adc2_buffer[3] = {0U, 0U, 0U};
float power_integral = 0.0f;
float global_phase;
float global_control_effort;
float global_clip_reason;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* ---------------- GPIO / power stage ---------------- */
#define CHARGER_ENABLE_PORT          GPIOB
#define CHARGER_ENABLE_PIN           GPIO_PIN_11
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
VofaData_t g_vofa_frame;
SystemState_t current_state = SYSTEM_OFF;

/* Raw ADC data */
uint16_t adc_pa6 = 0;
uint16_t adc_pc4 = 0;
uint16_t adc_pc0 = 0;
uint16_t adc_pa2 = 0;
uint16_t adc_pa3 = 0;



// Define a structure to hold your PID state
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
} PID_Controller;

PID_Controller power_pid = {0.1f, 0.05f, 0.0f, 0.0f, 0.0f};
/* Converted pin voltages */
float v_pa6 = 0.0f;
float v_pc4 = 0.0f;
float v_pc0 = 0.0f;
float v_pa2 = 0.0f;
float v_pa3 = 0.0f;

/* Raw reconstructed values */
float bus_voltage_raw = 0.0f;
float cap_voltage_raw = 0.0f;
float i_motor_current_raw = 0.0f;
float i_bat_current_raw   = 0.0f;
float i_cap_current_raw   = 0.0f;

/* Filtered values */
float bus_voltage = 0.0f;
float cap_voltage = 0.0f;
float i_motor_current = 0.0f;
float i_bat_current   = 0.0f;
float i_cap_current   = 0.0f;

/* Protection-path filtered absolute currents */
float i_bat_protect = 0.0f;
float i_cap_protect = 0.0f;

/* Init flag */
uint8_t filter_initialized = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void VOFA_Init(void);
void VOFA_UpdateData(void);
void VOFA_SendData(void);

void PowerStage_SetDutyPermille(uint32_t duty_permille);
void PowerStage_Enable(uint8_t en);

float INA240_VoltageToCurrent(float vout);
float ABSF(float x);

uint32_t DutyPermilleToCompare(TIM_HandleTypeDef *htim, uint32_t duty_permille);

//for power calc
#define BUFFER_SIZE 64  // x samples
float energy_buffer[BUFFER_SIZE] = {0};
uint8_t buffer_index = 0;
float current_power_watts = 0.0f;
//end power calc

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define FILTER_SIZE 16
float power_history[FILTER_SIZE];
int filter_index = 0;

float Get_Filtered_Power(float new_power) {
    // Add new sample to history
    power_history[filter_index] = new_power;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    // Calculate average
    float sum = 0.0f;
    for(int i = 0; i < FILTER_SIZE; i++) {
        sum += power_history[i];
    }
    return sum / FILTER_SIZE;
}
float ABSF(float x)
{
    return (x < 0.0f) ? -x : x;
}

float INA240_VoltageToCurrent(float voltage)
{
    return -1 * (((voltage - INA240_OFFSET_V) * CURRENT_SHUNT_SCALE) + CURRENT_OFFSET);
}

void PowerStage_Enable(uint8_t en)
{
    HAL_GPIO_WritePin(CHARGER_ENABLE_PORT,
                      CHARGER_ENABLE_PIN,
                      en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void VOFA_Init(void)
{
    g_vofa_frame.i_Motor = 0.0f;
    g_vofa_frame.V_Motor = 0.0f;
    g_vofa_frame.i_Bat   = 0.0f;
    g_vofa_frame.V_Cap   = 0.0f;
    g_vofa_frame.i_Cap   = 0.0f;
    g_vofa_frame.system_status = 0.0f;
    g_vofa_frame.phase = 0.0f;
    g_vofa_frame.control_effort = 0.0f;
    g_vofa_frame.clip_reason = 0.0f;
    g_vofa_frame.current_power = 0.0f;
    g_vofa_frame.tail    = 0x7F800000;
}

void VOFA_UpdateData(void)
{
    g_vofa_frame.i_Motor = i_motor_current;
    g_vofa_frame.V_Motor = bus_voltage;
    g_vofa_frame.i_Bat   = i_bat_current;
    g_vofa_frame.V_Cap   = cap_voltage;
    g_vofa_frame.i_Cap   = i_cap_current;
    g_vofa_frame.system_status = (float)current_state;
    g_vofa_frame.phase = global_phase;
    g_vofa_frame.control_effort = global_control_effort;
    g_vofa_frame.clip_reason = global_clip_reason;
    g_vofa_frame.current_power = current_power_watts;
    g_vofa_frame.tail    = 0x7F800000;

    global_clip_reason = 0.0f;
}

void VOFA_SendData(void)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)&g_vofa_frame, sizeof(g_vofa_frame), 1000);
}
void Process_Dynamic_Power_Tracking(float target_power, float v_cap, float measured_power, float v_bus)
{
    // 1. Feed-Forward Calculation
    float target_current = target_power / v_cap;
    float delta_v = target_current * SYSTEM_R;

    float ccr_neutral = (v_cap / v_bus) * TIMER_PERIOD;
    float ccr_feedforward = (delta_v / v_bus) * TIMER_PERIOD;

    // 2. PID Correction
    float error = target_power - measured_power;

    // Simple PI implementation (Integral + Proportional)
    float delta_t = CONTROL_DELAY_MS / 1000.0f;
    power_pid.integral += error * delta_t; // 0.001 represents your loop delta_t (e.g., 1ms)

    // Anti-windup: limit the integral so it doesn't grow to infinity
    if (power_pid.integral > 500.0f) power_pid.integral = 500.0f;
    if (power_pid.integral < -500.0f) power_pid.integral = -500.0f;

    float pid_correction = (power_pid.Kp * error) + (power_pid.Ki * power_pid.integral);//sign might be wrong

    // 3. Combine and Apply
    float final_ccr = ccr_neutral + ccr_feedforward + pid_correction;

    // 4. Safety Limits
    if (final_ccr > (TIMER_PERIOD * 0.95f)) final_ccr = (TIMER_PERIOD * 0.95f);
    if (final_ccr < 0.0f) final_ccr = 0.0f;

    // 5. Update Hardware
    global_phase = (uint32_t)final_ccr;
    TIM1->CCR1 = (uint32_t)final_ccr;
}
/* Decide what the supercap should do and the duty cycle to achieve that */
void PowerSharingControl(float bat_voltage, float bat_current, float cap_voltage, float cap_current, float motor_current, float motor_voltage)
{
	float requested_power = motor_current * motor_voltage;
	//float battery_power = bat_voltage * bat_current;
	float capacitor_energy = 0.5f * CAPACITANCE_TOTAL * cap_voltage * cap_voltage;
	float capacitor_power = cap_voltage * cap_current;

	//figure out mode
	if(ref_system_indicates_off){
		//off
		PowerStage_Enable(0); //turns off charging and discharging
		power_integral = 0.0f;
		current_state = SYSTEM_OFF;
	}
	else{
		//charging or discharging
		float target_power = POWER_LIMIT - requested_power;
		float safe_cap_volt = (cap_voltage > 0.5f) ? cap_voltage : 0.5f;
		float max_safe_charge_power = MAX_CHARGE_CURRENT * safe_cap_volt;

		//safety limits
		if ((capacitor_energy < MINIMUM_CAPACITOR_ENERGY) && (target_power < 0.0f)) {
			target_power = 0.0f;
			global_clip_reason += 4;
		}
		if ((cap_voltage > CAPACITOR_VOLTAGE_MAX) && (target_power > 0.0f)) {
			target_power = 0.0f;
			global_clip_reason += 8;
		}
		if((target_power > 0.0f) && (target_power > max_safe_charge_power)){
			target_power = max_safe_charge_power;
			global_clip_reason += 16;
		}
		if((target_power < 0) && (-target_power > MAX_DISCHARGE_CURRENT * cap_voltage)){
			target_power = -MAX_DISCHARGE_CURRENT * cap_voltage;
			global_clip_reason += 32;
		}
		global_control_effort =  target_power;
		Process_Dynamic_Power_Tracking(target_power, cap_voltage, capacitor_power, bat_voltage);
		PowerStage_Enable(1);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN2_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* 1. Arm the listener */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 5);

  /* 2. Put the Slave at attention */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* 3. Pull the Master trigger */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2); /* Starts the TRGO broadcast */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
  TIM3->CCR4 = 3646;
  while (1)
  {
     /* ===== READ ADCs ===== */
    uint16_t raw_imotor = adc1_buffer[0]; // Channel 1 (PA6)
    uint16_t raw_vbus   = adc1_buffer[1]; // Channel 2 (PC4)

    uint16_t raw_ibus = adc1_buffer[2];
    uint16_t raw_vcap = adc1_buffer[3];
    uint16_t raw_icap = adc1_buffer[4];

    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK)
    {
    	uint16_t raw_vrefint = HAL_ADC_GetValue(&hadc3);
    	// Use the built-in ST Low-Layer macro to compute the exact VREF voltage dynamically
    	dynamic_vref = (float)__LL_ADC_CALC_VREFANALOG_VOLTAGE(raw_vrefint, LL_ADC_RESOLUTION_16B) / 1000.0f;// may need to be _12B
    }
    HAL_ADC_Stop(&hadc3);

    v_pa6 = ADC_To_Voltage((float)raw_imotor, dynamic_vref);
    v_pc4 = ADC_To_Voltage((float)raw_vbus, dynamic_vref);
    v_pc0 = ADC_To_Voltage((float)raw_ibus, dynamic_vref);
    v_pa2 = ADC_To_Voltage((float)raw_vcap, dynamic_vref);
    v_pa3 = ADC_To_Voltage((float)raw_icap, dynamic_vref);

    /* Reconstruct bus voltage (no filtering) */
    bus_voltage_raw = (v_pc4 < ADC_ZERO_CLAMP_V) ? 0.0f : v_pc4 * BUS_SENSE_GAIN;

    /* Convert INA240 output to current (no filtering) */
    i_motor_current_raw = INA240_VoltageToCurrent(v_pa6);

    /* Reconstruct cap voltage (with filtering) */
//    float instant_vcap = (v_pa2 < ADC_ZERO_CLAMP_V) ? 0.0f : (v_pa2 * CAP_SENSE_GAIN);
    //cap_voltage_raw = (FILTER_ALPHA * instant_vcap) + ((1.0f - FILTER_ALPHA) * cap_voltage_raw);
    cap_voltage_raw = (v_pa2 < ADC_ZERO_CLAMP_V) ? 0.0f : (v_pa2 * CAP_SENSE_GAIN);

    /* Convert INA240 outputs to current (with filtering) */
    float instant_ibat = INA240_VoltageToCurrent(v_pc0);
    i_bat_current_raw = (FILTER_ALPHA * instant_ibat) + ((1.0f - FILTER_ALPHA) * i_bat_current_raw);

    float instant_icap = INA240_VoltageToCurrent(v_pa3);
    i_cap_current_raw = (FILTER_ALPHA * instant_icap) + ((1.0f - FILTER_ALPHA) * i_cap_current_raw);

    bus_voltage      = bus_voltage_raw;
    cap_voltage      = cap_voltage_raw;
    i_motor_current  = i_motor_current_raw;
    i_bat_current    = i_bat_current_raw;
    i_cap_current    = i_cap_current_raw;
    i_bat_protect    = i_bat_current_raw;
    i_cap_protect    = i_cap_current_raw;

    /* ===== PARALLEL POWER-SHARING CONTROLLER ===== */
    //PowerSharingControl(bus_voltage, i_bat_current, cap_voltage, i_cap_current, i_motor_current, bus_voltage);
    //the below code is for testing to make sure it can be charged and discahrged, once this is confirmed to work, comment it out and uncomment the above
    float v_cap = cap_voltage;

    // 2. Calculate the Neutral Duty Cycle (ensure 24.0 is a float to prevent integer truncation)
    float duty_neutral = v_cap / 24.0f;

    // 3. Convert the duty cycle percentage into the actual timer register value
    uint32_t ccr_neutral = (uint32_t)(duty_neutral * 3838.0f);

    // 4. Now apply your offset to actually move current!
    // To Charge (push current into the cap)
    TIM1->CCR1 = ccr_neutral + 75;

    /* ===== VOFA telemetry ===== */
    VOFA_UpdateData();
    VOFA_SendData();

    HAL_Delay(CONTROL_DELAY_MS);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
