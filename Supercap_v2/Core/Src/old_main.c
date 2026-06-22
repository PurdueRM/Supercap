///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body with parallel power-sharing system
//  ******************************************************************************
//  */
///* USER CODE END Header */
//
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "adc.h"
//#include "fdcan.h"
//#include "memorymap.h"
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include <stdint.h>
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//typedef struct
//{
//    float i_Motor;
//    float V_Motor;
//    float i_Bat;
//    float V_Cap;
//    float i_Cap;
//    uint32_t tail;
//} VofaData_t;
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* ---------------- GPIO / power stage ---------------- */
//#define CHARGER_ENABLE_PORT          GPIOB
//#define CHARGER_ENABLE_PIN           GPIO_PIN_11
//
///* ---------------- PWM settings ---------------- */
//#define PWM_DUTY_PERMILLE_MIN        0U
//#define PWM_DUTY_PERMILLE_START      80U
//#define PWM_DUTY_PERMILLE_ABS_MAX    500U
//#define CHARGE_DUTY_PERMILLE_MAX     400U
//#define DISCHARGE_DUTY_PERMILLE_MAX  450U
//#define CHARGE_DUTY_STEP_UP          20U
//#define CHARGE_DUTY_STEP_DOWN        20U
//#define DISCHARGE_DUTY_STEP_UP       30U
//#define DISCHARGE_DUTY_STEP_DOWN     20U
//#define DUTY_STEP_FAST_DOWN           40U
//
///* ---------------- Loop timing ---------------- */
//#define CONTROL_DELAY_MS             2U
//
///* ---------------- ADC scaling ---------------- */
//#define BUS_SENSE_GAIN               10.0f
//#define CAP_SENSE_GAIN               10.0f
//#define ADC_ZERO_CLAMP_V             0.05f
//
///* ---------------- INA240 current sense ---------------- */
//#define INA240_SHUNT_OHM             0.002f
//#define INA240_GAIN_V_PER_V          50.0f
//#define IMOTOR_ZERO_CURRENT_V        1.6987f
//#define IBAT_ZERO_CURRENT_V          1.7028f
//#define ICAP_ZERO_CURRENT_V          1.7042f
//
///* ---------------- Filters ---------------- */
//#define CURRENT_FILTER_ALPHA         0.08f
//#define VOLTAGE_FILTER_ALPHA         0.05f
//#define PROTECTION_FILTER_ALPHA      0.25f
//#define IMOTOR_FILTER_ALPHA          0.01f
//#define IMOTOR_DEADBAND_A            0.30f
//#define IMOTOR_AVG_SAMPLES           8U
//
///* ---------------- Power Sharing ---------------- */
//#define BATTERY_POWER_LIMIT_W        60.0f   // Hard battery power limit (W)
//#define MIN_BUS_VOLTAGE              1.0f    // Voltage floor to avoid divide-by-zero
//#define AUTO_CAP_ASSIST_MIN_V        16.0f  // Min cap voltage to assist
//
//#define DISCHARGE_CURRENT_HARD_A 5.5f
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
///* USER CODE BEGIN PV */
//
//VofaData_t g_vofa_frame;
//
///* Raw ADC data */
//uint16_t adc_pa6 = 0;
//uint16_t adc_pc4 = 0;
//uint16_t adc_pc0 = 0;
//uint16_t adc_pa2 = 0;
//uint16_t adc_pa3 = 0;
//
///* Converted pin voltages */
//float v_pa6 = 0.0f;
//float v_pc4 = 0.0f;
//float v_pc0 = 0.0f;
//float v_pa2 = 0.0f;
//float v_pa3 = 0.0f;
//
///* Raw reconstructed values */
//float bus_voltage_raw = 0.0f;
//float cap_voltage_raw = 0.0f;
//float i_motor_current_raw = 0.0f;
//float i_bat_current_raw   = 0.0f;
//float i_cap_current_raw   = 0.0f;
//
///* Filtered values */
//float bus_voltage = 0.0f;
//float cap_voltage = 0.0f;
//float i_motor_current = 0.0f;
//float i_bat_current   = 0.0f;
//float i_cap_current   = 0.0f;
//
///* Protection-path filtered absolute currents */
//float i_bat_protect = 0.0f;
//float i_cap_protect = 0.0f;
//
///* Duty command in permille */
//uint32_t pwm_duty_permille = PWM_DUTY_PERMILLE_START;
//
///* Init flag */
//uint8_t filter_initialized = 0U;
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//void PeriphCommonClock_Config(void);
//static void MPU_Config(void);
//
///* USER CODE BEGIN PFP */
//void VOFA_Init(void);
//void VOFA_UpdateData(void);
//void VOFA_SendData(void);
//
//void PowerStage_SetDutyPermille(uint32_t duty_permille);
//void PowerStage_Enable(uint8_t en);
//
//float INA240_VoltageToCurrent(float vout, float zero_v);
//float LPF_Apply(float prev, float input, float alpha);
//float ABSF(float x);
//float Read_Averaged_PA6_Voltage(uint8_t samples);
//
//uint32_t DutyPermilleToCompare(TIM_HandleTypeDef *htim, uint32_t duty_permille);
//
//void MPU_Config(void)
//{
//  MPU_Region_InitTypeDef MPU_InitStruct = {0};
//
//  HAL_MPU_Disable();
//
//  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
//  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
//  MPU_InitStruct.BaseAddress = 0x0;
//  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
//  MPU_InitStruct.SubRegionDisable = 0x87;
//  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
//  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
//  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
//  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
//  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
//  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
//
//  HAL_MPU_ConfigRegion(&MPU_InitStruct);
//  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
//}
//
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
//  RCC_OscInitStruct.HSICalibrationValue = 64;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = 4;
//  RCC_OscInitStruct.PLL.PLLN = 12;
//  RCC_OscInitStruct.PLL.PLLP = 1;
//  RCC_OscInitStruct.PLL.PLLQ = 2;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
//  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//  RCC_OscInitStruct.PLL.PLLFRACN = 0;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
//void PeriphCommonClock_Config(void)
//{
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
//  PeriphClkInitStruct.PLL2.PLL2M = 4;
//  PeriphClkInitStruct.PLL2.PLL2N = 12;
//  PeriphClkInitStruct.PLL2.PLL2P = 4;
//  PeriphClkInitStruct.PLL2.PLL2Q = 2;
//  PeriphClkInitStruct.PLL2.PLL2R = 2;
//  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
//  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
//  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
//  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
//void Error_Handler(void)
//{
//  __disable_irq();
//  while (1)
//  {
//  }
//}
//
//
///* NEW: true parallel power-sharing system */
//void PowerSharingControl(float vbus, float vcap, float ibat_raw, float icap_raw);
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//float LPF_Apply(float prev, float input, float alpha)
//{
//    return prev + alpha * (input - prev);
//}
//
//float ABSF(float x)
//{
//    return (x < 0.0f) ? -x : x;
//}
//
//float INA240_VoltageToCurrent(float vout, float zero_v)
//{
//    return (vout - zero_v) / (INA240_GAIN_V_PER_V * INA240_SHUNT_OHM);
//}
//
//float Read_Averaged_PA6_Voltage(uint8_t samples)
//{
//    uint32_t sum = 0U;
//    uint16_t pa6_tmp = 0U;
//    uint16_t pc4_dummy = 0U;
//    uint8_t i;
//
//    if (samples == 0U)
//    {
//        samples = 1U;
//    }
//
//    for (i = 0U; i < samples; i++)
//    {
//        ADC1_Read_PA6_PC4(&pa6_tmp, &pc4_dummy);
//        sum += pa6_tmp;
//    }
//
//    return ADC_To_Voltage((uint16_t)(sum / samples), 3.3f);
//}
//
//uint32_t DutyPermilleToCompare(TIM_HandleTypeDef *htim, uint32_t duty_permille)
//{
//    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
//
//    if (duty_permille > 1000U)
//    {
//        duty_permille = 1000U;
//    }
//
//    return ((arr + 1U) * duty_permille) / 1000U;
//}
//
//void PowerStage_SetDutyPermille(uint32_t duty_permille)
//{
//    uint32_t ccr1 = DutyPermilleToCompare(&htim1, duty_permille);
//    uint32_t ccr4 = DutyPermilleToCompare(&htim3, duty_permille);
//
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr4);
//
//    pwm_duty_permille = duty_permille;
//}
//
//void PowerStage_Enable(uint8_t en)
//{
//    HAL_GPIO_WritePin(CHARGER_ENABLE_PORT,
//                      CHARGER_ENABLE_PIN,
//                      en ? GPIO_PIN_SET : GPIO_PIN_RESET);
//}
//
//void VOFA_Init(void)
//{
//    g_vofa_frame.i_Motor = 0.0f;
//    g_vofa_frame.V_Motor = 0.0f;
//    g_vofa_frame.i_Bat   = 0.0f;
//    g_vofa_frame.V_Cap   = 0.0f;
//    g_vofa_frame.i_Cap   = 0.0f;
//    g_vofa_frame.tail    = 0x7F800000;
//}
//
//void VOFA_UpdateData(void)
//{
//    g_vofa_frame.i_Motor = i_motor_current;
//    g_vofa_frame.V_Motor = bus_voltage;
//    g_vofa_frame.i_Bat   = i_bat_current;
//    g_vofa_frame.V_Cap   = cap_voltage;
//    g_vofa_frame.i_Cap   = i_cap_current;
//    g_vofa_frame.tail    = 0x7F800000;
//}
//
//void VOFA_SendData(void)
//{
//    HAL_UART_Transmit(&huart4, (uint8_t *)&g_vofa_frame, sizeof(g_vofa_frame), 1000);
//}
//
///* ===================== TRUE PARALLEL POWER-SHARING ===================== */
//void PowerSharingControl(float vbus, float vcap, float ibat_raw, float icap_raw)
//{
//    float ibat_abs = ABSF(ibat_raw);
//    float icap_abs = ABSF(icap_raw);
//    float pwm_command = (float)pwm_duty_permille;
//
//    /* --- HARD BATTERY POWER LIMIT --- */
//    if (vbus > MIN_BUS_VOLTAGE)
//    {
//        float ibat_max = BATTERY_POWER_LIMIT_W / vbus;
//
//        if (ibat_abs > ibat_max)
//        {
//            float reduction_factor = ibat_max / ibat_abs;
//            pwm_command = pwm_command * reduction_factor;
//
//            if (pwm_command < PWM_DUTY_PERMILLE_MIN)
//            {
//                pwm_command = PWM_DUTY_PERMILLE_MIN;
//            }
//        }
//    }
//
//    /* --- CAPACITOR ASSIST --- */
//    if ((ibat_abs > (BATTERY_POWER_LIMIT_W / vbus)) && (vcap >= AUTO_CAP_ASSIST_MIN_V))
//    {
//        float extra_duty = (ibat_abs - (BATTERY_POWER_LIMIT_W / vbus)) / (DISCHARGE_CURRENT_HARD_A) * DISCHARGE_DUTY_STEP_UP;
//        pwm_command += extra_duty;
//
//        if (pwm_command > DISCHARGE_DUTY_PERMILLE_MAX)
//        {
//            pwm_command = DISCHARGE_DUTY_PERMILLE_MAX;
//        }
//    }
//
//    /* --- Apply PWM --- */
//    PowerStage_SetDutyPermille((uint32_t)pwm_command);
//    PowerStage_Enable(1);
//}
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  MPU_Config();
//  HAL_Init();
//
//  SystemClock_Config();
//  PeriphCommonClock_Config();
//
//  MX_GPIO_Init();
//  MX_ADC1_Init();
//  MX_ADC2_Init();
//  MX_FDCAN2_Init();
//  MX_UART4_Init();
//  MX_TIM1_Init();
//  MX_TIM3_Init();
//
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//
//  PowerStage_SetDutyPermille(PWM_DUTY_PERMILLE_START);
//  PowerStage_Enable(1);
//
//  VOFA_Init();
//
//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
//  HAL_Delay(200);
//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
//  HAL_Delay(200);
//
//  while (1)
//  {
//    /* ===== READ ADCs ===== */
//    ADC1_Read_PA6_PC4(&adc_pa6, &adc_pc4);
//    ADC2_Read_PC0_PA2_PA3(&adc_pc0, &adc_pa2, &adc_pa3);
//
//    /* Extra averaging for motor current */
//    v_pa6 = Read_Averaged_PA6_Voltage(IMOTOR_AVG_SAMPLES);
//
//    v_pc4 = ADC_To_Voltage(adc_pc4, 3.3f);
//    v_pc0 = ADC_To_Voltage(adc_pc0, 3.3f);
//    v_pa2 = ADC_To_Voltage(adc_pa2, 3.3f);
//    v_pa3 = ADC_To_Voltage(adc_pa3, 3.3f);
//
//    /* Reconstruct bus/cap voltages */
//    bus_voltage_raw = (v_pc4 < ADC_ZERO_CLAMP_V) ? 0.0f : v_pc4 * BUS_SENSE_GAIN;
//    cap_voltage_raw = (v_pa2 < ADC_ZERO_CLAMP_V) ? 0.0f : v_pa2 * CAP_SENSE_GAIN;
//
//    /* Convert INA240 outputs to currents */
//    i_motor_current_raw = INA240_VoltageToCurrent(v_pa6, IMOTOR_ZERO_CURRENT_V);
//    i_bat_current_raw   = INA240_VoltageToCurrent(v_pc0, IBAT_ZERO_CURRENT_V);
//    i_cap_current_raw   = INA240_VoltageToCurrent(v_pa3, ICAP_ZERO_CURRENT_V);
//
//    /* Motor deadband */
//    if ((i_motor_current_raw > -IMOTOR_DEADBAND_A) && (i_motor_current_raw < IMOTOR_DEADBAND_A))
//    {
//        i_motor_current_raw = 0.0f;
//    }
//
//    /* Initialize or filter */
//    if (!filter_initialized)
//    {
//        bus_voltage      = bus_voltage_raw;
//        cap_voltage      = cap_voltage_raw;
//        i_motor_current  = i_motor_current_raw;
//        i_bat_current    = i_bat_current_raw;
//        i_cap_current    = i_cap_current_raw;
//        i_bat_protect    = ABSF(i_bat_current_raw);
//        i_cap_protect    = ABSF(i_cap_current_raw);
//        filter_initialized = 1U;
//    }
//    else
//    {
//        bus_voltage      = LPF_Apply(bus_voltage, bus_voltage_raw, VOLTAGE_FILTER_ALPHA);
//        cap_voltage      = LPF_Apply(cap_voltage, cap_voltage_raw, VOLTAGE_FILTER_ALPHA);
//
//        i_motor_current  = LPF_Apply(i_motor_current, i_motor_current_raw, IMOTOR_FILTER_ALPHA);
//        i_bat_current    = LPF_Apply(i_bat_current, i_bat_current_raw, CURRENT_FILTER_ALPHA);
//        i_cap_current    = LPF_Apply(i_cap_current, i_cap_current_raw, CURRENT_FILTER_ALPHA);
//
//        i_bat_protect    = LPF_Apply(i_bat_protect, ABSF(i_bat_current_raw), PROTECTION_FILTER_ALPHA);
//        i_cap_protect    = LPF_Apply(i_cap_protect, ABSF(i_cap_current_raw), PROTECTION_FILTER_ALPHA);
//    }
//
//    /* ===== PARALLEL POWER-SHARING CONTROLLER ===== */
//    PowerSharingControl(bus_voltage, cap_voltage, i_bat_current_raw, i_cap_current_raw);
//
//    /* ===== VOFA telemetry ===== */
//    VOFA_UpdateData();
//    VOFA_SendData();
//
//    HAL_Delay(CONTROL_DELAY_MS);
//  }
//}
