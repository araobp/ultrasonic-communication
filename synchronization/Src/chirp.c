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

float32_t ref_chirp[PCM_SAMPLES * 2] = { 0.0f };

void generate_ref_chirp(float *ref_chirp, bool up, float sampling_rate, float phase) {

  float freq;
  float arg;
  float t = 0.0;

  float time_frame = (float)PCM_SAMPLES/(float)sampling_rate;
  float delta_f = (float)(F2 - F1)/time_frame;
  float delta_t = time_frame/(time_frame * (float)sampling_rate);
  uint32_t re, im;

  for (uint32_t i = 0; i< PCM_SAMPLES; i++) {
    if (up) freq = F1 + delta_f * t;  // Up chirp
    else freq = F2 - delta_f * t;  // Down chirp
    arg = 2.0 * PI * freq * t + phase;
    t = t + delta_t;
    re = i * 2;
    im = re + 1;
    ref_chirp[re] = arm_cos_f32(arg) * AMPLITUDE;
    ref_chirp[im] = arm_sin_f32(arg) * AMPLITUDE;
    //printf("i: %d, t: %f, freq: %f, theta: %f, cos_val: %f\n", i, t, freq, theta, cos_val);
  }
}

void init_ref_chirp(float sampling_rate) {
  generate_ref_chirp(ref_chirp, false, sampling_rate, -PI/2.0);  // Down-chirp
}

void mult_ref_chirp(float32_t *pSrc, float32_t *pDst) {
  arm_cmplx_mult_cmplx_f32(pSrc, ref_chirp, pDst, PCM_SAMPLES * 2);
}

void mult_ref_chirp_sim(float32_t *pDst) {
  mult_ref_chirp(ref_chirp, pDst);
}
