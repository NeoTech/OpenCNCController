# STM32 HAL Firmware

OpenCNC motion controller firmware for STM32F4/G4/H7 series.

## Supported MCUs

- STM32F401/F411 (Nucleo-F401RE)
- STM32F446 (high-performance)
- STM32G431/G474 (CORDIC, FMAC)
- STM32H743/H753 (ultimate performance)

## Features

- Hardware timer step generation (168+ MHz)
- DMA-driven step pulses
- USB Full Speed CDC
- FreeRTOS multitasking

## Requirements

- STM32CubeIDE or CMake + arm-none-eabi-gcc
- STM32 HAL drivers
- FreeRTOS

## Project Setup with STM32CubeMX

1. Create new project for your MCU
2. Enable:
   - USB_OTG_FS with CDC class
   - TIM2 for step generation
   - TIM3 for spindle PWM
   - FreeRTOS (CMSIS-RTOS v2)
3. Configure GPIO pins per your board
4. Generate code
5. Add OpenCNC source files

## Pin Configuration (Example - Nucleo-F401RE)

| Function | Pin | AF/Mode |
|----------|-----|---------|
| Step X | PA0 | Output PP |
| Dir X | PA1 | Output PP |
| Step Y | PA2 | Output PP |
| Dir Y | PA3 | Output PP |
| Step Z | PA4 | Output PP |
| Dir Z | PA5 | Output PP |
| Limit X | PB0 | Input PU |
| Limit Y | PB1 | Input PU |
| Limit Z | PB2 | Input PU |
| Probe | PB3 | Input PU |
| E-Stop | PB4 | Input PU |
| Spindle PWM | PC6 | TIM3_CH1 |
| Spindle EN | PC0 | Output PP |

## Building with CMake

```bash
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi.cmake ..
make -j
```

## Flashing

```bash
# Using ST-Link
st-flash write build/opencnc_hal.bin 0x8000000

# Using OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program build/opencnc_hal.elf verify reset exit"
```

## Performance

| MCU | Max Step Rate | Notes |
|-----|---------------|-------|
| F401 (84 MHz) | 150 kHz | Basic |
| F446 (180 MHz) | 250 kHz | Recommended |
| H743 (480 MHz) | 500 kHz | Premium |

## Architecture

```
┌─────────────────────────────────────────┐
│              Application                │
├─────────────────┬───────────────────────┤
│   Motion Task   │    Status Task        │
│   (Realtime)    │    (Normal)           │
├─────────────────┴───────────────────────┤
│              FreeRTOS                   │
├─────────────────────────────────────────┤
│           STM32 HAL/LL                  │
├─────────────────────────────────────────┤
│    Timer  │   GPIO   │   USB CDC        │
└─────────────────────────────────────────┘
```
