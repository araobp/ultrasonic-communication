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

float32_t up_chirp[PCM_SAMPLES * 2];
float32_t down_chirp[PCM_SAMPLES * 2];

void generate_ref_chirp(float *ref_chirp, int updown, float sampling_rate, float phase) {
  float32_t sin_val;
  float32_t cos_val;
  float freq;
  float theta;
  float t = 0.0;
  int re, im;

  float delta_f = (float)(F1 - F0)/TIME_FRAME;
  float delta_t = TIME_FRAME/(TIME_FRAME * sampling_rate);

  for (int i = 0; i< PCM_SAMPLES; i++) {
    if (updown == UP_CHIRP) {
      freq = F0 + delta_f * t /2.0;  // Up chirp
    } else if (updown == DOWN_CHIRP) {
      freq = F1 - delta_f * t / 2.0;    // Down chirp
    } else {
      freq = 0.0f;
    }
    theta = 360.0 * freq * t + phase;
    t = t + delta_t;
    arm_sin_cos_f32(theta, &sin_val, &cos_val);
    re = i * 2;
    im = re + 1;
    ref_chirp[re] = cos_val * AMPLITUDE;
    ref_chirp[im] = sin_val * AMPLITUDE;
    //printf("i: %d, t: %f, freq: %f, theta: %f, cos_val: %f, sin_val: %f\n", i, t, freq, theta, cos_val, sin_val);
  }
}

void init_ref_chirp(float sampling_rate) {
  generate_ref_chirp(up_chirp, UP_CHIRP, sampling_rate, -90.0);
  generate_ref_chirp(down_chirp, DOWN_CHIRP, sampling_rate, -90.0);
}

void mult_ref_chirp(float32_t *pInOut, int updown) {
  if (updown == UP_CHIRP) {
    arm_cmplx_mult_cmplx_f32(pInOut, up_chirp, pInOut, PCM_SAMPLES);
  } else if (updown == DOWN_CHIRP) {
    arm_cmplx_mult_cmplx_f32(pInOut, down_chirp, pInOut, PCM_SAMPLES);
  }
}

void mult_ref_chirp_sim(float32_t *pInOut, int updown) {
//  arm_cmplx_mult_cmplx_f32(up_chirp, down_chirp, pInOut, PCM_SAMPLES);
//  arm_cmplx_mult_cmplx_f32(up_chirp, up_chirp, pInOut, PCM_SAMPLES);
  pInOut = up_chirp;
}
