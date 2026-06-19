/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
float ADC_To_Voltage(float adc_value, float vref)
{
  return (adc_value * vref) / 65535.0f;
}

DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /* Rank 1 = PA6 = ADC1_INP3 */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Rank 2 = PC4 = ADC1_INP4 */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;

  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Rank 1 = PC0 = ADC2_INP10 */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Rank 2 = PA2 = ADC2_INP14 */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Rank 3 = PA3 = ADC2_INP15 */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (adcHandle->Instance == ADC1)
  {
    /* 1. Peripheral and Core Clock Enables */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if (HAL_RCC_ADC12_CLK_ENABLED == 1)
    {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_DMA1_CLK_ENABLE(); // Enable the DMA1 clock manually
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 2. GPIO Analog Configuration */
    /* PA6 -> ADC1_INP3 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PC4 -> ADC1_INP4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 3. Manual DMA1_Channel1 Configuration for ADC1 */
    hdma_adc1.Instance = DMA1_Stream0;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 16-bit counts
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    // 16-bit buffer
    hdma_adc1.Init.Mode = DMA_CIRCULAR;                           // Endlessly auto-refresh
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;                  // High priority for PWM midpoint sync

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    /* 4. Associate DMA handle with ADC handle */
    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
  }
  else if (adcHandle->Instance == ADC2)
  {
    /* 1. Peripheral and Core Clock Enables */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if (HAL_RCC_ADC12_CLK_ENABLED == 1)
    {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_DMA1_CLK_ENABLE(); // Ensure DMA1 clock is enabled here as well
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 2. GPIO Analog Configuration */
    /* PC0 -> ADC2_INP10 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PA2, PA3 -> ADC2_INP14, ADC2_INP15 */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 3. Manual DMA1_Channel2 Configuration for ADC2 */
    hdma_adc2.Instance = DMA1_Stream1; // Separate channel to avoid resource clash
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_MEDIUM; // Medium priority for free-running loop

    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    /* 4. Associate DMA handle with ADC handle */
    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc2);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{
  if (adcHandle->Instance == ADC1)
  {
    HAL_RCC_ADC12_CLK_ENABLED--;
    if (HAL_RCC_ADC12_CLK_ENABLED == 0)
    {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
  }
  else if (adcHandle->Instance == ADC2)
  {
    HAL_RCC_ADC12_CLK_ENABLED--;
    if (HAL_RCC_ADC12_CLK_ENABLED == 0)
    {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
