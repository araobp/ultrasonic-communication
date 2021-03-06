/*
 * chirp.h
 *
 *  Created on: 2018/06/07
 */

#ifndef CHIRP_H_
#define CHIRP_H_

// Amplitude of reference chirp
#define AMPLITUDE 1.0f;

// Chirp sweep range
#define F1 17000
#define F2 18000

void init_ref_chirp(float sampling_rate);
void mult_ref_chirp(float32_t *pSrc, float32_t *pDst);
void mult_ref_chirp_sim(float32_t *pDst);

#endif /* CHIRP_H_ */
