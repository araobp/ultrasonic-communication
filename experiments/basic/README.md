# Basic FFT code for MEMS microphones

The code is based on https://github.com/y2kblog/NUCLEO-L476RG_DFSDM_PDM-Mic with the following modifications:

- Use TrueSTUDIO instead of SW4STM32.
- DFSDM-related parameters on CubeMX.
- Support two MEMS microphones.
- Output single-shot data when the user button is pressed.
- Output FFT, raw data and filtered data.
