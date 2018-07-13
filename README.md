# Learning DSP with ultrasonic communications

![](./doc/receiver.jpg)

## Background and motivation

STMicro gave me STM32L4(Arm Cortex-M4) evaluation board and a pair of MEMS microphones for free at [a trade show held in Makuhari city](https://www.st.com/content/st_com/en/about/events/events.html/techno-frontier-2018.html), Chiba, Japan. Thanks a lot to STMicro!

As an IoT hobyyist, I am becoming interested in DSP with MEMS mic.

Also thanks to this: https://github.com/y2kblog/NUCLEO-L476RG_DFSDM_PDM-Mic.

## Project goal

I am a DSP beginner, so I learn DSP by doing: realize "low-power narrow area networking" by cheap DSP and MEMS mic.

## Project documentation

==> [Specification](https://docs.google.com/presentation/d/e/2PACX-1vSd3PQnKqmKbjcGNyNh_gygd175jgfzZYH5iwcEPqmmgiy7k3yYzqqHzfs7u-95jm_9hHgc0ugAvv2U/pub?start=false&loop=false&delayms=3000)

## Platform: STM32L4 platform and FFT test code on MEMS mic

==> [Platform](PLATFORM.md)

==> [Test code](./experiments/basic)

## Mathematical formula expressing ultrasonic wave

I tested frequency-hopping to transmit data over ultra-sonic, resulting in very bad performance at low SNR in a noisy room. So I decided to employ chirp modulation in this project.

![](./doc/orthogonal_upchirp.jpg)

==> [Formula](./misc/Formula.ipynb)

## Current work

### Simulation of time-frame synchronization

Unsynchronized chirp results in two peaks in frequency domain. Assuming that the clock accuracy of the transmitter and the receiver is bad, sync position adjustment is required even after synchronization.

==> [Simulation](./simulation/ChirpSynchronization.ipynb)

### Simulation of orthogonal chirp

Since I/Q modulation code did not fit into RAM of STM32, I am trying orthogonal chirp instead.

==> [Simulation](./simulation/OrthogonalChirp.ipynb)

### Experiment of orthogonal chirp (June 29, 2018)

Very weak orthogonal chirp tone was transmitted to the receiver:

![](./doc/experiment.jpg)

The receiver could detect the signal and showed a strong peak of magnitude around zero Hz, as long as its time frame is in sync with the transmitter:

![](./doc/Experiment_orthogonal_upchirp_upchirp.jpg)

### Successful implementation of ultrasonic receiver (July 10, 2018)

I spent a day to know that 4 times FFT does not fit into 20.5msec time frame (2048 samples/100000Hz) the day before. So I modified the sampling rate to 80MHz /32 divider /32 decimation = 78125Hz, so T of 2048 samples corresponds to 26.2msec.

Weak "Hello World!" tone was sent to the receiver, and it could decode the signal and showed the message!

==> [Code](./synchronization)

### Developing my original MEMS mic shield

I have bought [this MEMS mic](http://akizukidenshi.com/catalog/g/gM-05577/): Knowles SPM0405HD4H. The spec is similar to the mic on the expansion board from STMicro. Although this one does not support ultrasonic, it should be OK.

![Knowles](./doc/Knowles.jpg)

I am currently developing my original shield with Knowles MEMS mic:

- Knowles MEMS mic
- Character LCD
- Two LEDs (blue and red)
- One tactile switche (reset)

![](./doc/expansion_board_circuit.jpg)

## Experiments I made over the past month

I have made several experiments over the past month to study how data can be transmitted over ultra-sonic wave: FSK, hopping and chirp. The conclusion is to try Chirp modulation to fight with noise.

#### Ultrasonic communications experiment (FSK modulation)

==> [Experiment](./experiments/EXPERIMENT.md)

==> [Test code](./experiments/ultracom)

#### Ultrasonic communications experiment (Chirp modulation)

![](./doc/Simulation_upchirp_upchirp.jpg)

==> [Experiment2](./experiments/EXPERIMENT2.md)

==> [Test code](./experiments/chirp)

#### Ultrasonic communications experiment (Chirp modulation with compression)

==> [Experiment3](./experiments/EXPERIMENT3.md)
