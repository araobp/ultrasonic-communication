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

const float delta_f = (float)(F2 - F1)/TQ;
const float delta_t = TQ/(TQ * SAMPLING_RATE);

void generate_ref_chirp(float *ref_chirp, bool up) {
  float32_t sin_val;
  float32_t cos_val;
  float freq;
  float theta;
  float t = 0.0;
  int re, im;

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

void init_ref_chirp(void) {
  generate_ref_chirp(up_chirp, true);
  generate_ref_chirp(down_chirp, false);
}

void mult_ref_chirp(float32_t *pSrc, float32_t *pDst) {
  int re, im;
  float a, b, c, d;

  for (int i = 0; i<PCM_SAMPLES; i++) {
    re = i * 2;
    im = re + 1;
    a = pSrc[re];
    b = pSrc[im];
    c = down_chirp[re];
    d = down_chirp[im];
    // (a + jb) * (c + jd)
    pDst[re] = a * c - b * d;
    pDst[im] = a * d + b * c;
  }
}

void mult_ref_chirp_sim(float32_t *pDst) {
  mult_ref_chirp(up_chirp, pDst);
}
