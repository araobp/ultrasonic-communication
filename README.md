# Ultrasonic communications (STM32L4 DSP w/ MEMS mic)

![ChirpFrame](./doc/ChirpFrame.jpg)

As an IoT hobyyist, I am becoming interested in developing an IoT demo using DSP with MEMS mic. This project is to study Arm Cortex-M4's DSP capabilities. STMicro gave me the STM32L4 and MEMS mic free at some trade show in Chiba, Japan. Thanks a lot to STMicro!

## Preparation: STM32L4 platform and FFT test code on MEMS mic

This project uses STM32L476RG as MCU/DSP and MP34ST01-M as MEMS microphone:

![platform](./doc/MEMSMIC_expansion_board.jpg)

The system architecture is as follows:

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=2268&h=567)

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

## Ultrasonic communications experiment (FSK modulation)

My first idea was FSK modulation of 18 frequencies (18 symboles) to send data on ultrasonic wave. The method worked very well in a silent room, but did not work in a noisy environment such as a meeting room. I had to come up with another approach, such as spread spectrum.

![](./doc/18symbols.jpg)

==> [Experiment](EXPERIMENT.md)

==> [Test code](./ultracom)


## Ultrasonic communications experiment (Chirp modulation)

I tested Chirp modulation as one of spread spectrum techniques. It worked! But the noise problem still remained, and the result was useless in a noisy room. I had to come up with a noise suppression technique such as Chirp compression.

![](./doc/Chirp.jpg)

==> [Experiment2](EXPERIMENT2.md)

==> [Test code](./chirp)

## Ultrasonic communications experiment (Chirp modulation with compression)

This time, I employ some compression technique, and chirp appears one time within Time Quantum (TQ), unlike the previous experiment.

### Simulation on Jupyter Notebook

I made [chirp compression simulator](./simulation/ChirpSimulation.ipynb) on Jupyter Notebook.

The two peaks below are constant noises and chirp signal is under noise level of white noise:

![upchirp_with_noise](./doc/Simulation_upchirp_with_noise.jpg)

Chirp is identifiable at around 0 Hz:

![upchirp_upchirp_conjugate](./doc/Simulation_upchirp_upchirp_conjugate.jpg)

Chirp is identifiable at around 35000 Hz:

![upchirp_downchirp](./doc/Simulation_upchirp_downchirp.jpg)

### The first experiment of up x down on STM32L4 DSP (June 10, 2018)

I transmitted very weak chirp signals to STM32L4 DSP with MEMS mic. It worked! But I observed two peaks most of time, since the FFT calculation was performed on a chirp signal split into two within TQ, since the time frame was not in sync between the transmitter and the receiver.

![upchirp_downchirp](./doc/FFT_upXdown.jpg)

### How much time does it take to compute complex FFT of 2048 samples?

The measured value is 3msec for each complex FFT of 2048 samples at 80MHz system clock. 2048 samples correspond to 20.5msec at 100kHz sampling rate, so 3msec is short enough compared to 20.5msec.

### Next steps

#### Frame synchronization

I have to come up with some frame synchronization technique: synchronization of TQ between the transmitter and the receiver.

#### sinc filter optimization (moving average)

sinc5 filter seems to be the best for this project to reduce high-frequency noises.

#### Bandpass filter design

Design bandpass filter to remove unnecessary noise.

Reference:
- https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.firwin.html
- http://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR.html#gae8fb334ea67eb6ecbd31824ddc14cd6a

## My original MEMS mic shield

I have bought [this MEMS mic](http://akizukidenshi.com/catalog/g/gM-05577/): Knowles SPM0405HD4H. The spec is similar to the mic on the expansion board from STMicro. Although this one does not support ultrasonic, it should be OK.

![Knowles](./doc/Knowles.jpg)

I am going to make my original shield with Knowles MEMS mic:

- Knowles MEMS mic
- LCDs
- LEDs
- Tactile switches
- CAN tranceiver
