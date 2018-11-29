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

float32_t up_chirp[NN];
float32_t down_chirp[NN];

void generate_ref_chirp(float *ref_chirp, chirp updown, float sampling_rate, float phase) {
  float32_t sin_val;
  float32_t cos_val;
  float freq;
  float theta;
  float t = 0.0;

  float delta_f = (float)(F1 - F0)/TIME_FRAME;
  float delta_t = TIME_FRAME/(TIME_FRAME * sampling_rate);

  for (int n = 0; n < NN; n++) {
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
    ref_chirp[n] = cos_val * AMPLITUDE;
    ref_chirp[n] = sin_val * AMPLITUDE;
  }
}

void init_ref_chirp(float sampling_rate) {
  generate_ref_chirp(up_chirp, UP_CHIRP, sampling_rate, -90.0);
  generate_ref_chirp(down_chirp, DOWN_CHIRP, sampling_rate, -90.0);
}

void mult_ref_chirp(float32_t *signal, chirp updown) {
  if (updown == UP_CHIRP) {
    arm_mult_f32(signal, up_chirp, signal, NN);
  } else if (updown == DOWN_CHIRP) {
    arm_mult_f32(signal, down_chirp, signal, NN);
  }
}

void mult_ref_chirp_sim(float32_t *signal, chirp updown) {
  signal = up_chirp;
}
