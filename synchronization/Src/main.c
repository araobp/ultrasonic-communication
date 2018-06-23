
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32l4xx_hal.h"
#include "dfsdm.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdbool.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "chirp.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// flag: "new PCM data has just been copied to buf"
bool new_pcm_data = false;

// Audio sample rate and period
float sampling_rate;
float sampling_period;

// DMA peripheral to memory buffer
int32_t buf[PCM_SAMPLES] = { 0 };

// PCM data store for further processing (FFT)
float pcm[PCM_SAMPLES * 2] = { 0.0f };

// FFT
float32_t inout[PCM_SAMPLES * 2] = { 0.0f };
float fft_window[PCM_SAMPLES * 2] = { 0.0f };
const float WINDOW_SCALE = 2.0f * M_PI / (float) PCM_SAMPLES;

// UART output flag
bool output_result = false;

// UART input
uint8_t rxbuf[1];
bool all_data = true;

struct history {
  float mag_max;
  float mag_max_left;
  float mag_max_right;
  uint32_t max_idx;
  uint32_t max_idx_left;
  uint32_t max_idx_right;
  uint32_t start_time;
  uint32_t finish_time;
};

struct history history[8];

uint32_t center;
uint32_t bandwidth;

uint32_t p_offset = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*
 * reference: https://www.keil.com/pack/doc/CMSIS/DSP/html/arm_fft_bin_example_f32_8c-example.html
 */
void fft(uint32_t idx) {

  uint32_t start_time = HAL_GetTick();

  // Execute real up-chirp w/ real noise * complex down-chirp
  mult_ref_chirp(inout);

  // Windowing
  arm_mult_f32(inout, fft_window, inout, PCM_SAMPLES * 2);

  // Execute complex FFT
  arm_cfft_f32(&arm_cfft_sR_f32_len2048, inout, 0, 1);

  // Calculate magnitude
  arm_cmplx_mag_f32(inout, inout, PCM_SAMPLES / 2);

  // Calculate max magnitude
  float mag_max;
  float mag_max_left;
  float mag_max_right;
  uint32_t max_idx;
  uint32_t max_idx_left;
  uint32_t max_idx_right;
  arm_max_f32(&inout[center - bandwidth*2], bandwidth*4, &mag_max, &max_idx);
  arm_max_f32(&inout[center - bandwidth*2], bandwidth*2, &mag_max_left, &max_idx_left);
  arm_max_f32(&inout[center], bandwidth*2, &mag_max_right, &max_idx_right);

  uint32_t finish_time = HAL_GetTick();

  history[idx].mag_max = mag_max;
  history[idx].mag_max_left = mag_max_left;
  history[idx].mag_max_right = mag_max_right;
  history[idx].max_idx = max_idx;
  history[idx].max_idx_left = max_idx_left;
  history[idx].max_idx_right = max_idx_right + bandwidth*2;
  history[idx].start_time = start_time;
  history[idx].finish_time = finish_time;
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
  int im, re;
  int p_turn = 0;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */

  // FFT sampling rate
  sampling_rate = SystemCoreClock / hdfsdm1_channel2.Init.OutputClock.Divider
      / hdfsdm1_filter0.Init.FilterParam.Oversampling
      / hdfsdm1_filter0.Init.FilterParam.IntOversampling;

  // Frequency range of peak magnitudes of compressed chirp
  center = (F1 + F2) * PCM_SAMPLES / sampling_rate ;
  bandwidth = (F2 - F1) * PCM_SAMPLES / sampling_rate;

  // Initialize reference chirp signal
  init_ref_chirp(sampling_rate);

  // Enable UART receive interrupt
  HAL_UART_Receive_IT(&huart2, rxbuf, 1);

  // Enable DMA from DFSDM to buf (peripheral to memory)
  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, buf, PCM_SAMPLES)
      != HAL_OK) {
    Error_Handler();
  }

  // Hanning window
  for (uint32_t i = 0; i < PCM_SAMPLES; i++) {
    re = i * 2;
    im = re + 1;
    fft_window[im] = 0.5f - 0.5f * arm_cos_f32((float) i * WINDOW_SCALE);
    fft_window[re] = fft_window[im];
  }

  // Show starting message
  printf("/// Receiver ///\n\n");
  printf("Sampling rate: %4.1f(kHz)\n", (float) sampling_rate / 1000.0f);
  sampling_period = 1.0f / sampling_rate * PCM_SAMPLES;
  printf("Sampling period: %.1f(msec), Samples per period: %ld\n\n",
      sampling_period * 1000.0f, PCM_SAMPLES);
  printf("Push USER button to output single-shot FFT\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t offset = PCM_SAMPLES / 8;
  uint32_t shift = PCM_SAMPLES / 4;

  while (1) {

    HAL_Delay(1);

    // Wait for next PCM samples from M1
    if (new_pcm_data) {

      for (uint32_t i = 0; i < 4; i++) {
        int sync_position = p_turn * offset + shift * i;
        for (uint32_t j = 0; j < PCM_SAMPLES; j++) {
          re = j * 2;
          im = re + 1;
          inout[re] = pcm[j + sync_position];
          inout[im] = 0.0f;
        }
        fft(i * 2 + p_turn);
      }
      p_turn = (p_turn == 0)? 1: 0;

      if (output_result && p_turn == 1) {  // Output debug info

        printf("Magnitude history:\n");
        for (int i = 0; i < 8; i++) {
          printf("max: %.1f, max_r: %.1f, max_l: %.1f, s_time: %lu, f_time: %lu, i: %lu, i_left: %lu, i_right: %lu\n",
              history[i].mag_max, history[i].mag_max_left, history[i].mag_max_right,
              history[i].start_time,
              history[i].finish_time, history[i].max_idx,
              history[i].max_idx_left, history[i].max_idx_right);
        }

        if (all_data) {
          printf("index,Magnitude\n");
          // FFT
          for (uint32_t i = 0; i < PCM_SAMPLES / 2; i++) {
            printf("%lu,%e\n", i, inout[i]);
          }
          printf("EOF\n");
        }

        // Filtered PCM data (e.g., Hanning Window)
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        output_result = false;
      }

      new_pcm_data = false;

    }
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

/* USER CODE BEGIN 4 */

/**
 * @brief  Half regular conversion complete callback.
 * @param  hdfsdm_filter : DFSDM filter handle.
 * @retval None
 */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(
    DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {

}

/**
 * @brief  Regular conversion complete callback.
 * @note   In interrupt mode, user has to read conversion value in this function
 using HAL_DFSDM_FilterGetRegularValue.
 * @param  hdfsdm_filter : DFSDM filter handle.
 * @retval None
 */
void HAL_DFSDM_FilterRegConvCpltCallback(
    DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
  if (!new_pcm_data && (hdfsdm_filter == &hdfsdm1_filter0)) {
    arm_copy_f32(&pcm[PCM_SAMPLES], pcm, PCM_SAMPLES);
    for (uint32_t i = 0; i < PCM_SAMPLES; i++) {
      pcm[i + PCM_SAMPLES] = (float) buf[i];
    }
    new_pcm_data = true;
  }
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *) ptr, (uint16_t) len, 0xFFFFFFFF);
  return len;
}

/**
 * @brief  GPIO External Interrupt callback
 * @param  GPIO_Pin
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_13) {  // User button (blue tactile switch)
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    output_result = true;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  switch(rxbuf[0]) {
  case '1':
    all_data = true;
    break;
  case '2':
    all_data = true;
    break;
  case '3':
    all_data = false;
    break;
  }

  output_result = true;

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart2, rxbuf, 1);
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
  printf("Error! Line = %d\n", line);
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
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
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
