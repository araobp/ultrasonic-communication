/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dfsdm.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdbool.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "chirp.h"
#include "lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

const uint32_t NN_HALF = NN / 2;
const uint32_t NN_DOUBLE = NN * 2;

// flag: "new PCM data has just been copied to buf"
bool new_pcm_data = false;

// Audio sampling rate and period
float fs;
float T;

// DMA peripheral to memory buffer
int32_t buf[NN] = { 0 };

// FIFO queue of PCM data for further processing (FFT)
float fifo_queue[NN * 3] = { 0.0f };

// FFT-related
arm_rfft_fast_instance_f32 S;
float hann_window[NN];
const float WINDOW_SCALE = 2.0f * M_PI / (float) NN;

// UART output flag
bool output_result = false;

// UART input/output
uint8_t rxbuf[1];

// State machine
enum state {
  IDLE, SYNCHRONIZING, SYNCHRONIZED, DATA_RECEIVING
};
enum state state = IDLE;

const char STATE[4][16] = { "IDLE", "SYNCHRONIZING", "SYNCHRONIZED",
    "DATA_RECEIVING" };
const char SIMPLE_STATE[4] = { 'I', 'G', 'S', 'R' };

enum ope_mode {
  NORMAL, SIMPLE, DETAIL
};

const enum ope_mode MODE = DETAIL;

// Stats
struct history {
  float mag_max;  // Max magnitude in the bandwidth
  float mag_max_left;  // Max magnitude in the left
  float mag_max_right;  // Max magnitude in the right
  int32_t max_freq;  // Frequency at max magnitude in the bandwidth
  int32_t max_freq_left;   // Frequency at max magnitude in the left
  int32_t max_freq_right;  // Frequency at max magnitude in the right
  uint32_t start_time;   // DSP start time
  uint32_t finish_time;  // DSP finish time
  float mag_mean;  // Mean magnitude in the past
  float snr;  // S/N Ratio
  char rank;  // Magnitude ranking
};

uint32_t bandwidth;
uint32_t bandwidth2;
uint32_t idx_left_zero;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int32_t idx2freq(uint32_t idx) {
  if (idx < NN_HALF) {
    return (int32_t) fs * idx / NN;
  } else {
    return ((int32_t) fs * (NN - idx) / NN) * -1;
  }
}

// Digital signal processing pipeline
void pipeline(float *signal, int updown) {

  float32_t signal_buf[NN] = { 0.0f };

  // Execute real up-chirp w/ real noise * complex up-chirp
  mult_ref_chirp(signal, updown);

  // Windowing
  arm_mult_f32(signal, hann_window, signal, NN);

  // Execute real FFT
  arm_rfft_fast_f32(&S, signal, signal_buf, 0);
  arm_copy_f32(signal_buf, signal, NN);

  // Calculate magnitude
  arm_cmplx_mag_f32(signal, signal, NN);

}

// Digital signal processing
void dsp(uint32_t sync_position, struct history *phist, float32_t mag_mean,
    int updown) {

  float32_t time_frame[NN_DOUBLE];

  float mag_max;
  float mag_max_left;
  float mag_max_right;
  uint32_t max_idx;
  uint32_t max_idx_left;
  uint32_t max_idx_right;

  // Copy PCM data to input/output buffer for DSP
  for (uint32_t n = 0; n < NN; n++) {
    time_frame[n] = fifo_queue[sync_position + n];
  }

  uint32_t start_time = HAL_GetTick();

// Digital signal processing pipeline
  pipeline(time_frame, updown);

// Calculate indexes of max magnitudes
  arm_max_f32(&time_frame[idx_left_zero], bandwidth2, &mag_max_left,
      &max_idx_left);
  arm_max_f32(&time_frame[0], bandwidth2, &mag_max_right, &max_idx_right);
  if (mag_max_left > mag_max_right) {
    mag_max = mag_max_left;
    max_idx = idx_left_zero + max_idx_left;
  } else {
    mag_max = mag_max_right;
    max_idx = max_idx_right;
  }

  uint32_t finish_time = HAL_GetTick();

  // Stats
  phist->mag_max = mag_max;
  phist->mag_max_left = mag_max_left;
  phist->mag_max_right = mag_max_right;
  phist->max_freq = idx2freq(max_idx);
  phist->max_freq_left = idx2freq(idx_left_zero + max_idx_left);
  phist->max_freq_right = idx2freq(max_idx_right);
  phist->start_time = start_time;
  phist->finish_time = finish_time;
  phist->mag_mean = mag_mean;
  phist->snr = (mag_max - mag_mean) / mag_mean;

}

float32_t symbol_snr(uint32_t sync_position, struct history *phist, int updown) {
  dsp(sync_position, phist, phist->mag_mean, updown);
  return phist->snr;
}

void set_ranks(struct history hist[], char r0, char r1) {
  hist[0].rank = r0;
  hist[1].rank = r1;
}

void resync(float32_t snr, struct history hist[], uint32_t offset,
    uint32_t *sync_position, int updown) {

  int32_t sync_position_l = *sync_position - offset;
  int32_t sync_position_r = *sync_position + offset;

  float32_t snr_l = symbol_snr(sync_position_l, &hist[2], updown);
  float32_t snr_r = symbol_snr(sync_position_r, &hist[3], updown);

  if ((snr > snr_l) && (snr > snr_r)) {
    set_ranks(&hist[2], '-', '-');
  } else {  // Adjust sync_position

    if (snr_l >= snr_r) {
      if (sync_position_l >= 0) {
        *sync_position = sync_position_l;
        set_ranks(&hist[2], 'L', '-');  // move sync_position left
      } else {
        set_ranks(&hist[2], 'l', '-');  // reached left edge
      }
    } else if (snr_l < snr_r) {
      if (sync_position_r <= NN_DOUBLE) {
        *sync_position = sync_position_r;
        set_ranks(&hist[2], '-', 'R');  // move sync_position right
      } else {
        set_ranks(&hist[2], '-', 'r');  // reached right edge
      }
    }

  }
}

void print_history(enum state prev_state, enum state state,
    struct history hist[], int num, enum ope_mode mode) {

  switch (mode) {

  case NORMAL:
    break;

  case SIMPLE:
    printf("%c => %c\n", SIMPLE_STATE[prev_state], SIMPLE_STATE[state]);
    for (int i = 0; i < num; i++) {
      printf("%c,%6.1f\n", hist[i].rank, hist[i].snr);
    }
    break;

  case DETAIL:
    printf("\nstate: %s => %s\n", STATE[prev_state], STATE[state]);
    printf(
        "r,  freq,freq_l,freq_r, t_s, t_f,      max,    max_l,    max_r, mag_mean,   snr\n");
    for (int i = 0; i < num; i++) {
      printf("%c,%6ld,%6ld,%6ld,%4lu,%4lu, %4.2e, %4.2e, %4.2e, %4.2e,%6.1f\n",
          hist[i].rank, hist[i].max_freq, hist[i].max_freq_left,
          hist[i].max_freq_right, hist[i].start_time % 1000,
          hist[i].finish_time % 1000, hist[i].mag_max, hist[i].mag_max_left,
          hist[i].mag_max_right, hist[i].mag_mean, hist[i].snr);
    }
    break;
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  uint32_t max_idx = 0;
  uint32_t turn = 0;
  uint32_t offset, shift;

// Stats-related
  struct history history[8];

  float mag_stat[12] = { 1E37, 1E37, 1E37, 1E37, 1E37, 1E37, 1E37, 1E37, 1E37,
      1E37, 1E37, 1E37 };

  float mag_max;
  float mag_mean;
  float mag_max_max;

  // Time frame synchronization
  uint32_t sync_cnt = 0;
  uint32_t sync_position = NN_HALF;
  float32_t snr, snr_up, snr_down;

  enum state prev_state = IDLE;

  char msg = 0U;
  int msg_cnt = 0;
  char line[17];
  int line_cnt = 0;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */

// FFT sampling rate
  fs = SystemCoreClock / hdfsdm1_channel5.Init.OutputClock.Divider
      / hdfsdm1_filter0.Init.FilterParam.Oversampling
      / hdfsdm1_filter0.Init.FilterParam.IntOversampling;

// Statistics-related
  bandwidth = (F1 - F0) * NN / fs;
  bandwidth2 = bandwidth * 2;
  idx_left_zero = NN - bandwidth2;

  // Real FFT initialization
  arm_rfft_fast_init_f32(&S, NN);

// Initialize reference chirp signal
  init_ref_chirp(fs);

// Enable UART receive interrupt
  HAL_UART_Receive_IT(&huart2, rxbuf, 1);

// Enable DMA from DFSDM to buf (peripheral to memory)
  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, buf, NN) != HAL_OK) {
    Error_Handler();
  }

// Generate Hanning window
  for (uint32_t i = 0; i < NN; i++) {
    hann_window[i] = 0.5f - 0.5f * arm_cos_f32((float) i * WINDOW_SCALE);
  }

// Print starting message
  printf("/// Receiver ///\n\n");
  printf("Sampling rate: %4.1f(kHz)\n", (float) fs / 1000.0f);
  T = 1.0f / fs * NN;
  printf("Sampling period: %.1f(msec), Samples per period: %ld\n\n",
      T * 1000.0f, NN);
  printf("Push USER button to output single-shot FFT\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  offset = NN / 8;
  shift = NN / 4;

  /* LCD initialization */
  lcd_init(&hi2c1);
  lcd_string("Receiver start", 15);

  /* LED initialization */
  int led_state = GPIO_PIN_SET;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, led_state);

  while (1) {

    HAL_Delay(1);

    // Wait for next PCM samples from DFSDM (MEMS mic)
    if (new_pcm_data) {

      prev_state = state;

      switch (state) {

      case IDLE:

        sync_cnt = 0;
        arm_mean_f32(&mag_stat[4], 8, &mag_mean);
        led_state = GPIO_PIN_SET;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, led_state);
        // intentionally no break here

      case SYNCHRONIZING:

        // Execute FFT four times per time frame (every 20.5msec)
        //
        //             sync_position = turn * offset + shift * i
        //             |
        //             V
        // data        <-2048 samples->
        // turn 0  |---|---|---|---|
        // turn 1  : |---|---|---|---|
        //          -> offset
        for (uint32_t i = 0; i < 4; i++) {
          sync_position = NN_HALF + turn * offset + shift * i;
          int idx = i * 2 + turn;  // Stats index
          dsp(sync_position, &history[idx], mag_mean, UP_CHIRP);
        }

        turn = (turn == 0) ? 1 : 0;

        if (turn == 1) {

          // Shift magnitude history
          for (int i = 10; i >= 0; i--) {
            mag_stat[i + 1] = mag_stat[i];
          }

          // Find max magnitude in the history
          mag_max_max = 0.0f;
          for (int i = 0; i < 8; i++) {
            history[i].rank = '-';
            mag_max = history[i].mag_max;
            if (mag_max > mag_max_max) {
              mag_max_max = mag_max;
              max_idx = i;
            }
          }

          mag_stat[0] = mag_max_max;  // TODO: is this necessary?
          history[max_idx].rank = '+';

          // S/N Ratio
          snr = (mag_max_max - mag_mean) / mag_mean;

          if (snr >= SNR_THRESHOLD) {
            state = SYNCHRONIZING;
            if (++sync_cnt >= 3) {
              state = SYNCHRONIZED;
              sync_position = NN_HALF + max_idx * offset;
            }
          } else {
            state = IDLE;
          }
        }
        break;

      case SYNCHRONIZED:

        snr_up = symbol_snr(sync_position, &history[0], UP_CHIRP);
        snr_down = symbol_snr(sync_position, &history[1], DOWN_CHIRP);

        if ((snr_up >= SNR_THRESHOLD) || (snr_down >= SNR_THRESHOLD)) {

          if (snr_down > snr_up) {
            set_ranks(history, '-', 'D');
            resync(snr_down, history, offset, &sync_position, DOWN_CHIRP);
            state = DATA_RECEIVING;
          } else {
            set_ranks(history, 'U', '-');
            resync(snr_up, history, offset, &sync_position, UP_CHIRP);
          }

        } else {
          state = IDLE;
        }
        break;

      case DATA_RECEIVING:

        led_state =
            (led_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, led_state);

        snr_up = symbol_snr(sync_position, &history[0], UP_CHIRP);
        snr_down = symbol_snr(sync_position, &history[1], DOWN_CHIRP);

        if ((snr_up >= SNR_THRESHOLD) || (snr_down >= SNR_THRESHOLD)) {

          if (snr_down > snr_up) {  // Bit low
            set_ranks(history, '-', '0');
            msg = (msg << 1) + 0;
            resync(snr_down, history, offset, &sync_position, DOWN_CHIRP);
          } else {  // Bit high
            set_ranks(history, '1', '-');
            msg = (msg << 1) + 1;
            resync(snr_up, history, offset, &sync_position, UP_CHIRP);
          }
          if (++msg_cnt >= 8) {
            printf("%c", msg);
            sprintf(&line[line_cnt++], "%c", msg);
            msg = 0U;
            msg_cnt = 0;
          }

        } else {
          printf("\n");
          line[line_cnt] = '\0';
          lcd_print_scroll(line);
          state = IDLE;
          msg = 0U;
          msg_cnt = 0;
          line_cnt = 0;
          led_state = GPIO_PIN_SET;
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, led_state);
        }
        break;

      default:
        break;
      }

      switch (prev_state) {
      case IDLE:
        if (state == SYNCHRONIZING) {
          //print_history(prev_state, state, history, 8, MODE);
        }
        break;
      case SYNCHRONIZING:
        if (turn == 1) {  // Output debug info
          //print_history(prev_state, state, history, 8, MODE);
        }
        break;
      case SYNCHRONIZED:
        //print_history(prev_state, state, history, 4, MODE);
        break;
      case DATA_RECEIVING:
        //print_history(prev_state, state, history, 4, MODE);
        break;
      default:
        break;
      }

      new_pcm_data = false;

    }
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

  /**Initializes the CPU, AHB and APB busses clocks 
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1
      | RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }
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
    arm_copy_f32(&fifo_queue[NN], fifo_queue, NN_DOUBLE);
    for (uint32_t i = 0; i < NN; i++) {
      fifo_queue[NN_DOUBLE + i] = (float) buf[i];
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

  switch (rxbuf[0]) {
    default:
    break;
  }

  output_result = true;

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart2, rxbuf, 1);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
