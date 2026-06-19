/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with parallel power-sharing system
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal_rcc_ex.h"
#include "adc.h"
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "config.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    float i_Motor;
    float V_Motor;
    float i_Bat;
    float V_Cap;
    float i_Cap;
    uint32_t tail;
} VofaData_t;

uint16_t adc1_buffer[2] = {0U, 0U}; //holds adc values for motor current and bus voltage
// Index 0 = PC0 (IBUS)
// Index 1 = PA2 (VCAP)
// Index 2 = PA3 (ICAP)
uint16_t adc2_buffer[3] = {0U, 0U, 0U};
float power_integral = 0.0f;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------------- GPIO / power stage ---------------- */
#define CHARGER_ENABLE_PORT          GPIOB
#define CHARGER_ENABLE_PIN           GPIO_PIN_11

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

VofaData_t g_vofa_frame;

/* Raw ADC data */
uint16_t adc_pa6 = 0;
uint16_t adc_pc4 = 0;
uint16_t adc_pc0 = 0;
uint16_t adc_pa2 = 0;
uint16_t adc_pa3 = 0;

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

void MX_DMA_Init(void)
{
  /* 1. Enable the clock for the DMAMUX and the DMA engine */
	RCC->AHB3ENR |= (1UL << 28);
	__HAL_RCC_DMA1_CLK_ENABLE();

  /* 2. Configure the DMA Interrupt Priorities */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

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
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

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

void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

void PowerSharingControl(float bat_voltage, float bat_current, float cap_voltage, float cap_current, float motor_current, float motor_voltage);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float ABSF(float x)
{
    return (x < 0.0f) ? -x : x;
}

float INA240_VoltageToCurrent(float voltage)
{
    return voltage * CURRENT_SHUNT_SCALE;
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
    g_vofa_frame.tail    = 0x7F800000;
}

void VOFA_UpdateData(void)
{
    g_vofa_frame.i_Motor = i_motor_current;
    g_vofa_frame.V_Motor = bus_voltage;
    g_vofa_frame.i_Bat   = i_bat_current;
    g_vofa_frame.V_Cap   = cap_voltage;
    g_vofa_frame.i_Cap   = i_cap_current;
    g_vofa_frame.tail    = 0x7F800000;
}

void VOFA_SendData(void)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)&g_vofa_frame, sizeof(g_vofa_frame), 1000);
}


float Process_Dynamic_Power_Tracking(float target_power, float actual_power)
{
    static float last_target_power = 0.0f;
    //positive target power = charging

    // 1. Calculate the moving physical baseline
    float equilibrium_ff = cap_voltage_raw / bus_voltage_raw;
    float power_ff = 0.0f;
    if (cap_voltage_raw > 1.0f) {
            power_ff = (SYSTEM_R * target_power) / (cap_voltage_raw * bus_voltage_raw);
    }

    float combined_feed_forward = equilibrium_ff + power_ff;

    // 2. Calculate actual power
    float power_error = target_power - actual_power;

    // If the mode changes, clear mem to prevent lag/overshoot
    if ((target_power > 0.0f && last_target_power < 0.0f) ||
        (target_power < 0.0f && last_target_power > 0.0f))
    {
        power_integral = 0.0f;
    }
    last_target_power = target_power;

    power_integral += power_error * KI_GAIN;

    // clamp integral term
    if (power_integral > MAX_INTEGRAL_TERM)  power_integral = MAX_INTEGRAL_TERM;
    if (power_integral < -MAX_INTEGRAL_TERM) power_integral = -MAX_INTEGRAL_TERM;

    // 6. Compute final duty cycle
    float final_duty = combined_feed_forward + (power_error * KP_GAIN) + power_integral;

    //make sure charging always charges under the available power
    //and make sure discharging always charges over the necessary extra power
    if (target_power > 0.0f)
        {
            // --- CHARGING MODE ---
            // Never exceed target_power
            if (actual_power >= target_power && final_duty > combined_feed_forward)
            {
                final_duty = combined_feed_forward; // Force it back toward 0A charging
                power_integral = 0.0f;          // Freeze integral windup
            }
        }
    else if (target_power < 0.0f)
        {
            // --- DISCHARGING MODE ---
            // Never discharge "more" than target_power
            if (actual_power <= target_power && final_duty < combined_feed_forward)
            {
                final_duty = combined_feed_forward; // Force it back toward 0A discharging
                power_integral = 0.0f;          // Freeze integral windup
            }
        }

    return final_duty;
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
	}
	else{
		//charging or discharging
		float target_power = POWER_LIMIT - requested_power;

		//safety limits
		if ((capacitor_energy < MINIMUM_CAPACITOR_ENERGY) && (target_power < 0.0f)) {
			target_power = 0.0f;
		}
		if ((cap_voltage > CAPACITOR_VOLTAGE_MAX) && (target_power > 0.0f)) {
			target_power = 0.0f;
		}
		if((target_power > 0) && (target_power > MAX_CHARGE_CURRENT * cap_voltage)){
			target_power = MAX_CHARGE_CURRENT * cap_voltage;
		}
		if((target_power < 0) && (-target_power > MAX_DISCHARGE_CURRENT * cap_voltage)){
			target_power = -MAX_DISCHARGE_CURRENT * cap_voltage;
		}

		float final_duty = Process_Dynamic_Power_Tracking(target_power, capacitor_power);

		/* --- Apply PWM --- */
		if (final_duty > MAX_DUTY_CYCLE) final_duty = MAX_DUTY_CYCLE;
		if (final_duty < MIN_DUTY_CYCLE) final_duty = MIN_DUTY_CYCLE;

		uint32_t ccr_value = (uint32_t)(final_duty * 3839.0f);
		TIM1->CCR1 = ccr_value;
		TIM3->CCR4 = ccr_value;
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
  MPU_Config();
  HAL_Init();

  SystemClock_Config();
  PeriphCommonClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN2_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 2);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, 3);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //for duty cycle

  VOFA_Init();

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(200);

  while (1)
  {
    /* ===== READ ADCs ===== */
    uint16_t raw_imotor = adc1_buffer[0]; // Channel 1 (PA6)
    uint16_t raw_vbus   = adc1_buffer[1]; // Channel 2 (PC4)

    uint16_t raw_ibus = adc2_buffer[0];
    uint16_t raw_vcap = adc2_buffer[1];
    uint16_t raw_icap = adc2_buffer[2];

    float v_pa6 = ADC_To_Voltage((float)raw_imotor, 3.3f);
    float v_pc4 = ADC_To_Voltage((float)raw_vbus, 3.3f);
    float v_pc0 = ADC_To_Voltage((float)raw_ibus, 3.3f);
    float v_pa2 = ADC_To_Voltage((float)raw_vcap, 3.3f);
    float v_pa3 = ADC_To_Voltage((float)raw_icap, 3.3f);

    /* Reconstruct bus voltage (no filtering) */
    bus_voltage_raw = (v_pc4 < ADC_ZERO_CLAMP_V) ? 0.0f : v_pc4 * BUS_SENSE_GAIN;

    /* Convert INA240 output to current (no filtering) */
    i_motor_current_raw = INA240_VoltageToCurrent(v_pa6);

    /* Reconstruct cap voltage (with filtering) */
    float instant_vcap = (v_pa2 < ADC_ZERO_CLAMP_V) ? 0.0f : (v_pa2 * CAP_SENSE_GAIN);
    cap_voltage_raw = (FILTER_ALPHA * instant_vcap) + ((1.0f - FILTER_ALPHA) * cap_voltage_raw);

    /* Convert INA240 outputs to current (with filtering) */
    float instant_ibat = INA240_VoltageToCurrent(v_pc0);
    i_bat_current_raw = (FILTER_ALPHA * instant_ibat) + ((1.0f - FILTER_ALPHA) * i_bat_current_raw);

    float instant_icap = INA240_VoltageToCurrent(v_pa3);
    i_cap_current_raw = (FILTER_ALPHA * instant_icap) + ((1.0f - FILTER_ALPHA) * i_cap_current_raw);

    if (ABSF(i_motor_current_raw) < IMOTOR_DEADBAND_AMPS)
    	{
            i_motor_current_raw = 0.0f;
        }

    if (ABSF(i_cap_current_raw) < ICAP_DEADBAND_AMPS)
        {
            i_cap_current_raw = 0.0f;
        }

    if (ABSF(i_bat_current_raw) < IBAT_DEADBAND_AMPS)
        {
            i_bat_current_raw = 0.0f;
        }

    bus_voltage      = bus_voltage_raw;
    cap_voltage      = cap_voltage_raw;
    i_motor_current  = i_motor_current_raw;
    i_bat_current    = i_bat_current_raw;
    i_cap_current    = i_cap_current_raw;
    i_bat_protect    = i_bat_current_raw;
    i_cap_protect    = i_cap_current_raw;

    /* ===== PARALLEL POWER-SHARING CONTROLLER ===== */
    PowerSharingControl(bus_voltage, i_bat_current, cap_voltage, i_cap_current, i_motor_current, bus_voltage);

    /* ===== VOFA telemetry ===== */
    VOFA_UpdateData();
    VOFA_SendData();

    HAL_Delay(CONTROL_DELAY_MS);
  }
}
