# Ultrasonic communication by STM32L4 and MEMS microphone

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=2268&h=567)

## Preparation: STM32L4 platform and FFT test code on MEMS mic

This project uses STM32L476RG as MCU and MP34ST01-M as MEMS microphone.

![platform](./doc/MEMSMIC_expansion_board.jpg)

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

## Ultrasonic communications experiment

==> [Experiment](EXPERIMENT.md)

==> [Test code](./ultracom)

## Next step

- Add a character LDC to the board.
- 16bit-length data transmission by mixing different frequencies at tone generator.
