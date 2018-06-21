/*
 * chirp.c
 *
 *  Created on: 2018/06/07
 */

#include "arm_math.h"
#include "arm_const_structs.h"
#include "chirp.h"
#include "main.h"
#include "stdbool.h"

const float WINDOW_SCALE = 2.0f * PI / (float)(PCM_SAMPLES - 1);

arm_rfft_fast_instance_f32 S;

float fft_window[PCM_SAMPLES] = { 0.0f };

// FFT of reference up-chirp
float32_t H_up[PCM_SAMPLES] = { 0.0f };
// FFT of reference down-chirp
float32_t H_down[PCM_SAMPLES] = { 0.0f };
float32_t ref_chirp[PCM_SAMPLES] = { 0.0f };

void generate_ref_chirp(float *ref_chirp, bool up, float sampling_rate, float phase) {

  float freq;
  float arg;
  float t = 0.0;

  float time_frame = (float)PCM_SAMPLES / (float)sampling_rate;
  float delta_f = (F2 - F1) / time_frame;
  float delta_t = time_frame / (time_frame * (float)sampling_rate);

  float value;

  for (int i = 0; i< PCM_SAMPLES; i++) {
    if (up) freq = F1 + delta_f * t;  // Up chirp
    else freq = F2 - delta_f * t;  // Down chirp
    arg = 2.0 * PI * freq * t + phase;
    t = t + delta_t;
    value = arm_cos_f32(arg);
    ref_chirp[i] = value * AMPLITUDE;
  }
}

void windowing(float32_t *pInOut) {
  // Windowing
  arm_mult_f32(pInOut, fft_window, pInOut, PCM_SAMPLES);
}

void init_ref_chirp(float sampling_rate) {

  // Initialize CMSIS DSP Real FFT API
  arm_rfft_fast_init_f32(&S, PCM_SAMPLES);

  // Generate reference chirp
  generate_ref_chirp(H_up, true, sampling_rate, -PI/2.0);
  generate_ref_chirp(H_down, false, sampling_rate, -PI/2.0);
  generate_ref_chirp(ref_chirp, false, sampling_rate, -PI/2.0);

  // Generate Hanning window
  for (uint32_t i = 0; i < PCM_SAMPLES; i++) {
    fft_window[i] = 0.5f - 0.5f * arm_cos_f32((float)i * WINDOW_SCALE);
  }

  // Apply the window to reference down-chirp
  windowing(H_up);
  windowing(H_down);

  // Then FFT
  arm_rfft_fast_f32(&S, H_up, H_up, 0);
  arm_rfft_fast_f32(&S, H_down, H_down, 0);

}

// Compress chirp in time domain
void compress_chirp(float32_t *pInOut) {
  windowing(pInOut);
  arm_rfft_fast_f32(&S, pInOut, pInOut, 0);
  arm_cmplx_mult_cmplx_f32(pInOut, H_down, pInOut, PCM_SAMPLES/2);
  arm_rfft_fast_f32(&S, pInOut, pInOut, 1);  // IFFT
}

void compress_ref_chirp(float32_t *pInOut) {
  arm_cmplx_mult_cmplx_f32(H_up, H_down, pInOut, PCM_SAMPLES/2);
  arm_rfft_fast_f32(&S, pInOut, pInOut, 1);  // IFFT
}

void get_ref_chirp(float32_t *pInOut) {
  arm_copy_f32(H_up, pInOut, PCM_SAMPLES);
}

