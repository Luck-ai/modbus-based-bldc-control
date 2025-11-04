# Modbus-Based BLDC Motor Control

Field-Oriented Control (FOC) implementation for BLDC motors using STM32F401RE with Modbus communication support.

## Overview

This project implements FOC motor control for Brushless DC motors on the STM32 Nucleo-F401RE board. It uses an AS5048A magnetic encoder for position feedback, SSD1306 OLED display for visualization, and supports velocity/position control modes.

## Hardware

- **MCU**: STM32F401RE (Nucleo-F401RE)
- **Encoder**: AS5048A (SPI)
- **Display**: SSD1306 OLED (I2C)
- **Motor**: 3-phase BLDC motor

## Features

- Field-Oriented Control (FOC)
- PID velocity/position control
- AS5048A encoder support
- OLED display interface
- UART communication
- Modbus support (in development)

## Building

### Using CMake
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### Using STM32CubeIDE
1. Import project
2. Build (Project → Build Project)
3. Flash to board

## Credits

Based on [SimpleFOC](https://github.com/simplefoc/Arduino-FOC)


