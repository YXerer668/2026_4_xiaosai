# 2026_4_xiaosai

STM32F407ZGT6 adaptive filtering demo for a competition project.

## Hardware platform

- MCU: STM32F407ZGT6 / STM32F407ZGTx
- ADC1 PA0: mixed input signal
- ADC2 PA1: reference interference signal
- DAC PA4: filtered output
- PC13: reset/status LED
- PD13: debug GPIO

## Sampling chain

- ADC mode: ADC1 + ADC2 dual regular simultaneous mode
- ADC trigger: TIM3 TRGO
- DAC trigger: TIM2 TRGO
- Default ADC sample rate: 50 kHz
- Default DAC output rate: 200 kHz
- DAC interpolation factor: 4

With APB1 timer clock = 84 MHz:

```text
TIM3: 84 MHz / 20 / 84 = 50 kHz
TIM2: 84 MHz / 20 / 21 = 200 kHz
```

Keep the CubeMX `.ioc` timing values consistent with `Core/Src/tim.c`; otherwise regenerating code may silently break the sampling rate.

## Algorithm

The firmware currently contains:

- NLMS adaptive filtering
- reference signal dynamic notch filtering
- output frequency measurement
- square-wave reference detection
- half/full DMA buffer processing
- timing diagnostics using TIM5

## Build

```bash
make -j
```

## Debug variables

Useful watch variables:

- `adc_fs_half`
- `adc_fs_full`
- `nlms_time_us_half`
- `nlms_time_us_full`
- `debug_nlms_freq_hz`
- `debug_nlms_freq_confidence`

For stable real-time processing, keep:

```text
nlms_time_us_half < tim5_delta_half
```
