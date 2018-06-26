# Ultrasonic communications (Arm Cortex-M4 w/ MEMS mic)

![](./doc/chirp_x_carrier.jpg)

[Formula] Ultra-sonic wave to deliver data to the receiver

## Background and motivation

STMicro gave me STM32L4(Arm Cortex-M4) evaluation board and a pair of MEMS microphones for free at [a trade show held in Makuhari city](https://www.st.com/content/st_com/en/about/events/events.html/techno-frontier-2018.html), Chiba, Japan. Thanks a lot to STMicro!

As an IoT hobyyist, I am becoming interested in DSP with MEMS mic.

## Platform: STM32L4 platform and FFT test code on MEMS mic

This project uses STM32L476RG as MCU/DSP and MP34ST01-M as MEMS microphone:

![platform](./doc/MEMSMIC_expansion_board.jpg)

The system architecture is as follows:

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=2268&h=567)

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

## Experiments

I have made several experiments over the past month to study how data can be transmitted over ultra-sonic wave: FSK, hopping and chirp. The conclusion is to try Chirp modulation to fight with noise. I will make another experiment in the near future to test OFDM and CDMA as well.

==> [EXPERIMENTS](EXPERIMENTS.md)

## Current work

### IQ modulation

Try IQ modulation to see if SNR improves.

==> [Formula](./misc/Formula.ipynb)

==> [Simulation](./simulation/IQ_modulation.ipynb)

If IQ modulation improves SNR, I will also test low-pass filter (sinc FIR) on Arm Cortex-M4 DSP.

### Expressing data in symbol

This implementation use up-chirp and down-chirp to express 0 and 1:

- up-chirp means 0.
- down-chirp means 1.

So one symbol contains 1 bit info.

![](./doc/chirp_baseband.jpg)

### Improving SNR (signal-to-noise ratio)

- Bandwidth optimization.
- Synchronous addition: multiple sets of 2048 PCM samples.
- Hardware sinc filter optimization on DFSDM.

### Frame synchronization problems

Unsynchronized chirp results in two peaks.

==> [Simulation](./simulation/ChirpSynchronization.ipynb)

I have come up with the following method for synchronization:

![](https://docs.google.com/drawings/d/e/2PACX-1vT9da0oKUWgUHHTmYUO8Y0Rix6ORT5aeQxAz8Ihjoxc4vWMvFLudPTet1UHLMConm5RDk9kFaXTXnj8/pub?w=960&h=720)

Assuming that the clock accuracy of the transmitter and the receiver is bad, sync position adjustment is required even after synchronization, maybe every 8 time frames.

#### Sync experiment on June 24, 2018

I made the sync code. It seemed to work.

I observed that Magnitude of chirp at a synchronized position (around i=25) was significantly larger than the others. The result below also showed me that one chirp can express multiple symbols: for example, |up|(0), |down|(1), |up0:up1|(2), |down0:down1|(3).

```
Magnitude history:
max: 306946688.0, max_r: 306946688.0, max_l: 46697000.0, s_time: 301404, f_time: 301408, i: 14, i_left: 14, i_right: 78
max: 599477568.0, max_r: 599477568.0, max_l: 22733306.0, s_time: 301382, f_time: 301386, i: 20, i_left: 20, i_right: 41
max: 777540416.0, max_r: 777540416.0, max_l: 53296356.0, s_time: 301408, f_time: 301413, i: 25, i_left: 25, i_right: 40
max: 727269504.0, max_r: 727269504.0, max_l: 248580016.0, s_time: 301386, f_time: 301391, i: 30, i_left: 30, i_right: 40
max: 508824928.0, max_r: 508824928.0, max_l: 447869984.0, s_time: 301413, f_time: 301417, i: 35, i_left: 35, i_right: 42
max: 336429216.0, max_r: 159654848.0, max_l: 336429216.0, s_time: 301391, f_time: 301395, i: 47, i_left: 39, i_right: 47
max: 192139792.0, max_r: 22508948.0, max_l: 192139792.0, s_time: 301417, f_time: 301422, i: 55, i_left: 37, i_right: 55
max: 98077944.0, max_r: 75327512.0, max_l: 98077944.0, s_time: 301396, f_time: 301400, i: 66, i_left: 9, i_right: 66
```

## My original MEMS mic shield

I have bought [this MEMS mic](http://akizukidenshi.com/catalog/g/gM-05577/): Knowles SPM0405HD4H. The spec is similar to the mic on the expansion board from STMicro. Although this one does not support ultrasonic, it should be OK.

![Knowles](./doc/Knowles.jpg)

I am going to make my original shield with Knowles MEMS mic:

- Knowles MEMS mic
- LCDs
- LEDs
- Tactile switches
- CAN tranceiver
