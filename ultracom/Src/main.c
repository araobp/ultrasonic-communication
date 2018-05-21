
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
#include "stdlib.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// Flags to indicate ISR to copy data from buffer to PCM data store for FFT
char M1[] = "M1";  // MEMS mic 1
char M2[] = "M2";  // MEMS mic 2

char *mic_select;
bool flag_m1 = true;
bool flag_m2 = true;

// Audio sample rate and period
float sample_rate;
float sample_period;

// DMA peripheral to memory buffer
int32_t buf_m1[FFT_SAMPLES] = { 0 };
int32_t buf_m2[FFT_SAMPLES] = { 0 };

// PCM data store for further processing (FFT)
int32_t pcm_m1[FFT_SAMPLES] = { 0 };
int32_t pcm_m2[FFT_SAMPLES] = { 0 };

// FFT
arm_rfft_fast_instance_f32 S;

// Ultrasonic communication
typedef struct {
  float max_mag;
  float max_freq;
} FFT_Result;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
 * @brief  Check if the frequency observed is within the tolerance.
 * @param  freq_observed: frequency observed (max frequency).
 * @param  hex_freq: frequency corresponding to hex data.
 * @retval true if it is within the tolerance.
 */
bool check_freq(uint32_t freq_observed, uint32_t hex_freq) {
  return abs(freq_observed - hex_freq) < TOLERANCE;
}

/**
 * @brief  Convert observed frequency to hex data
 * @param  freq_max: frequency at max magnitude of FFT result
 * @retval hex data, START_OF_FRAME_CODE if it's start-of-frame,
 * END_OF_FRAME_CODE if it's end-of-frame, or
 * NOT_FOUND_CODE if it's not within the frequency range of hex data.
 */
uint8_t freq2hex(float freq_max) {
  uint32_t freq_observed = (uint32_t)freq_max;
  uint32_t frequencies[16] = { HEX_0_FREQ, HEX_1_FREQ, HEX_2_FREQ, HEX_3_FREQ, HEX_4_FREQ,
      HEX_5_FREQ, HEX_6_FREQ, HEX_7_FREQ, HEX_8_FREQ, HEX_9_FREQ, HEX_A_FREQ, HEX_B_FREQ,
      HEX_C_FREQ, HEX_D_FREQ, HEX_E_FREQ, HEX_F_FREQ };
  int left = 0;
  int right = 16;
  uint16_t mid;
  uint freq;
  uint16_t hex_found = NOT_FOUND_CODE;  // not found

  if (check_freq(freq_observed, START_OF_FRAME)) {
    hex_found = START_OF_FRAME_CODE;
  }
  else if (check_freq(freq_observed, END_OF_FRAME)) {
    hex_found = END_OF_FRAME_CODE;
  }
  else {  // binary search
    while(left <= right) {
        mid = (left + right) / 2;
        freq = frequencies[mid];
        if (check_freq(freq_observed, freq)) {
          hex_found = mid;
          break;
        } else if (freq < freq_max) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
  }
  return hex_found;
}

/**
 * @brief  FFT to frequency at max magnitude
 * @param
 * @retval result including max frequency and max magnitude.
 */
void fft(float *input, float *output, float *window, float *magnitude, float *frequency, FFT_Result* result) {

  float mag_max, freq_max;
  uint32_t maxIndex;

  // Windowing
  arm_mult_f32(input, window, input, FFT_SAMPLES);

  // Execute FFT
  arm_rfft_fast_f32(&S, input, output, 0);

  // Calculate magnitude
  arm_cmplx_mag_f32(output, magnitude, FFT_SAMPLES / 2);

  // Normalization (Unitary transformation) of magnitude
  arm_scale_f32(magnitude, 1.0f / sqrtf((float) FFT_SAMPLES), magnitude, FFT_SAMPLES / 2);

  // Cut off lower frequency components
  for (uint32_t i = 0; i < FFT_SAMPLES / 2; i++) {
    if (frequency[i] < FFT_CUT_OFF) {
      magnitude[i] = 1.0f;
    } else {
      break;
    }
  }

  // Calculate max magnitude
  arm_max_f32(magnitude, FFT_SAMPLES / 2, &mag_max, &maxIndex);
  freq_max = frequency[maxIndex];

  // Return the result
  result->max_mag = mag_max;
  result->max_freq = freq_max;
}

/**
 * @brief  Parse fft result to output received data in ASCII to USART and LCD
 * @param  result
 * @retval
 */
void parser(FFT_Result *result) {

  bool output_result = false;
  enum { IDLE, DATA_MSB, DATA_LSB } recv_state = IDLE;

  uint8_t hex_data;
  static uint8_t hex_data_3 = 0xff;
  static uint8_t data_cnt = 0;
  static uint8_t data_msb = 0;

  // Convert frequency to hex data
  if (result->max_mag > MAGNITUDE_THRESHOLD) {
    hex_data = freq2hex(result->max_freq);
    if (hex_data != hex_data_3) {
      data_cnt = 0;
      hex_data_3 = hex_data;
    } else if (data_cnt == 3) {
      // No operation
    } else if (++data_cnt == 3) {
      output_result = true;
    }
  }

  // Output result to UART when user button is pressed
  if (output_result) {
    printf("\nMEMS mic: %s\n", mic_select);
    printf("Frequency at max magnitude: %.1f, Max magnitude: %f\n", result->max_freq,
        result->max_mag);
    switch(hex_data_3) {
      case START_OF_FRAME_CODE:
        printf("Hex data: START OF FRAME\n");
        recv_state = DATA_MSB;
        break;
      case END_OF_FRAME_CODE:
        printf("Hex data: END OF FRAME\n");
        recv_state = IDLE;
        break;
      default:
        if (recv_state == DATA_MSB) {
          data_msb = hex_data_3 << 4;
          recv_state = DATA_LSB;
        } else if (recv_state == DATA_LSB) {
          printf("Hex data: %c\n", data_msb + hex_data_3);
          data_msb = 0;
          recv_state = DATA_MSB;
        }
        break;
    }
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    output_result = false;
  }
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
  float fft_input[FFT_SAMPLES] = { 0.0f };
  float fft_output[FFT_SAMPLES] = { 0.0f };
  float fft_frequency[FFT_SAMPLES / 2] = { 0.0f };
  float fft_magnitude[FFT_SAMPLES / 2] = { 0.0f };
  float fft_window[FFT_SAMPLES] = { 0.0f };
  const float WINDOW_SCALE = 2.0f * M_PI / (float) FFT_SAMPLES;

  FFT_Result fft_result;
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

  // DMA from DFSDM to buf_m1
  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, buf_m1, FFT_SAMPLES)
      != HAL_OK) {
    Error_Handler();
  }
  // DMA from DFSDM to buf_m2
  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter2, buf_m2, FFT_SAMPLES)
      != HAL_OK) {
    Error_Handler();
  }

  // FFT sample rate
  sample_rate = SystemCoreClock / hdfsdm1_channel2.Init.OutputClock.Divider
      / hdfsdm1_filter0.Init.FilterParam.Oversampling
      / hdfsdm1_filter0.Init.FilterParam.IntOversampling;

  // Output basic info
  printf("/// Ultrasonic communication receiver ///\n\n");
  printf("Sampling rate: %4.1f(kHz)\n", (float) sample_rate / 1000.0f);
  sample_period = 1.0f / sample_rate * FFT_SAMPLES;
  printf("Sampling period: %.1f(msec), Samples per period: %ld\n\n",
      sample_period * 1000.0f, FFT_SAMPLES);

  // Hanning window
  for (uint32_t i = 0; i < FFT_SAMPLES; i++) {
    fft_window[i] = 0.5f - 0.5f * arm_cos_f32((float)i * WINDOW_SCALE);
  }

  for (uint32_t i = 0; i < FFT_SAMPLES / 2; i++) {
    fft_frequency[i] = (float)i * (float)sample_rate / (float)FFT_SAMPLES;
  }

  // FFT initialization
  arm_rfft_fast_init_f32(&S, FFT_SAMPLES);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    // Wait for next PCM samples from M1
    if (!flag_m1) {
      for (uint32_t i = 0; i < FFT_SAMPLES; i++) {
        fft_input[i] = (float) pcm_m1[i];
      }
      mic_select = M1;
      fft(fft_input, fft_output, fft_window, fft_magnitude, fft_frequency, &fft_result);
      parser(&fft_result);
      flag_m1 = true;
    }
    // Wait for next PCM samples from M2
    if (!flag_m2) {
      for (uint32_t i = 0; i < FFT_SAMPLES; i++) {
        fft_input[i] = (float) pcm_m2[i];
      }
      mic_select = M2;
      fft(fft_input, fft_output, fft_window, fft_magnitude, fft_frequency, &fft_result);
      parser(&fft_result);
      flag_m2 = true;
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
  //
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
  // uint32_t start = FFT_SAMPLES / 2;
  uint32_t start = 0;
  if (flag_m1 && (hdfsdm_filter == &hdfsdm1_filter0)) {
    for (uint32_t i = start; i < FFT_SAMPLES; i++) {
      pcm_m1[i] = buf_m1[i];
    }
    flag_m1 = false;
  } else if (flag_m2 && (hdfsdm_filter == &hdfsdm1_filter2)) {
    for (uint32_t i = start; i < FFT_SAMPLES; i++) {
      pcm_m2[i] = buf_m2[i];
    }
    flag_m2 = false;
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
