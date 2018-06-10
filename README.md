# Ultrasonic communications by STM32L4's DSP and MEMS mic

![ChirpFrame](./doc/ChirpFrame.jpg)

## Preparation: STM32L4 platform and FFT test code on MEMS mic

This project uses STM32L476RG as MCU and MP34ST01-M as MEMS microphone:

![platform](./doc/MEMSMIC_expansion_board.jpg)

The system architecture is as follows:

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=2268&h=567)

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

## Ultrasonic communications experiment (FSK modulation)

==> [Experiment](EXPERIMENT.md)

==> [Test code](./ultracom)

Conclusion: the method (sort of FSK modulation) work very well in a silent room, but did not work in a noisy environment such as a meeting room. I have to come up with another approach, such as spread spectrum.

## Chirp modulation experiment

I tested Chirp modulation as one of spread spectrum techniques. It worked!

==> [Experiment2](EXPERIMENT2.md)

### Next step: fight with noise

I have made some simulation on Jupyter Notebook: https://github.com/araobp/ultrasonic-communication/blob/master/generator/ChirpSimulation.ipynb

Chirp signal is under noise level:

![upchirp_with_noise](./doc/Simulation_upchirp_with_noise.jpg)

Chirp is identifiable at around 0 Hz:

![upchirp_upchirp_conjugate](./doc/Simulation_upchirp_upchirp_conjugate.jpg)

Chirp is identifiable at around 35000 Hz:

![upchirp_downchirp](./doc/Simulation_upchirp_downchirp.jpg)

### The first experiment of up x down (June 10, 2018)

![upchirp_downchirp](./doc/FFT_upXdown.jpg)

## My original MEMS mic shield

I have bought [this MEMS mic](http://akizukidenshi.com/catalog/g/gM-05577/): Knowles SPM0405HD4H. The spec is similar to the mic on the expansion board from STMicro. Although this one does not support ultrasonic, it should be OK.

![Knowles](./doc/Knowles.jpg)

I am going to make my original shield with Knowles MEMS mic:

- Knowles MEMS mic
- LCDs
- LEDs
- Tactile switches
- CAN tranceiver
