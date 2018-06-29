# Basic FFT code for MEMS microphones

The code is based on https://github.com/y2kblog/NUCLEO-L476RG_DFSDM_PDM-Mic with the following modifications:

- Use TrueSTUDIO instead of SW4STM32.
- DFSDM-related parameters on CubeMX.
- Support two MEMS microphones.
- Output single-shot data when the user button is pressed.
- Output FFT, raw data and filtered data.

## Pitfalls

This DSP function modifies p as well:

```
void arm_rfft_fast_f32	(	arm_rfft_fast_instance_f32 * 	S,
float32_t * 	p,
float32_t * 	pOut,
uint8_t 	ifftFlag 
)	
```

https://www.keil.com/pack/doc/CMSIS/DSP/html/group__RealFFT.html#ga180d8b764d59cbb85d37a2d5f7cd9799
