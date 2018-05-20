# Ultrasonic communication by STM32L4 and MEMS microphone

![architecture](https://docs.google.com/drawings/d/e/2PACX-1vR1KKp2QeL_SmrnUsTl5zcwddQToPJmnSBHFnxiw78y3_3mjA7EzNl2iNcUA5aOW_jRAQapTNji-eJ7/pub?w=480&h=189)

## Platform and test code

This project uses STM32L476RG as MCU and MP34ST01-M as MEMS microphone.

==> [Platform](PLATFORM.md)

==> [Test code](./basic)

## Hex number in 8bit length

Although this is not ultrasonic, but I assign the following frequencies to each hexa-decimal number of 8 bit data length:

|Hz   |Hex|
|-----|---|
|18500| 0 |
|18600| 1 |
|18700| 2 |
|18800| 3 |
|18900| 4 |
|19000| 5 |
|19100| 6 |
|19200| 7 |
|19300| 8 |
|19400| 9 |
|19500| A |
|19600| B |
|19700| C |
|19800| D |
|19900| E |
|20000| F |

## DFSDM setting

|Parameter    |Value/setting|
|-------------|-----|
|System clock |80MHz|
|Clock divider|25   |
|Decimation   |32   |
|Filter       |sinc3|

The resulting sampling rate is 100kHz (Nyquist frequency is 50kHz). If the length of FFT input data is 2048, the data length corresponds 10msec.

```
    [5/A][A][A][A][A][A/B][B][B]..

```
If the FFT peak is same and its magnitude is larger than a specific value in three times in a row (30msec), the receiver recognize it as a meaningful hex data.

## Data frame

```
    [Start-of-frame][Identifier(4bit)][DLC(4bit)][D0(8bit)]...[Dn(8bit)][Enf-of-frame]
       18300 Hz                                                           18400Hz
       50msec                                                             50msec
```


