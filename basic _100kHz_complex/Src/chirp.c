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

float32_t up_chirp[PCM_SAMPLES * 2] = {0.0f};
float32_t down_chirp[PCM_SAMPLES * 2] = {0.0f};

void generate_ref_chirp(float *ref_chirp, bool up, float sampling_rate) {
  float32_t sin_val;
  float32_t cos_val;
  float freq;
  float theta;
  float t = 0.0;
  int re, im;

  float delta_f = (float)(F2 - F1)/TQ;
  float delta_t = TQ/(TQ * sampling_rate);

  for (int i = 0; i< PCM_SAMPLES; i++) {
    if (up) freq = F1 + delta_f * t;  // Up chirp
    else freq = F2 - delta_f * t;  // Down chirp
    theta = 360.0 * freq * t;
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
  generate_ref_chirp(up_chirp, true, sampling_rate);
  generate_ref_chirp(down_chirp, false, sampling_rate);
}

void mult_ref_chirp(float32_t *pSrc, float32_t *pDst) {
  arm_cmplx_mult_cmplx_f32(pSrc, down_chirp, pDst, PCM_SAMPLES);
}

void mult_ref_chirp_sim(float32_t *pDst) {
  mult_ref_chirp(up_chirp, pDst);
}
