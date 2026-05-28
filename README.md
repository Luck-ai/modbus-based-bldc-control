<div align="center">

# ⚡ Modbus-Based BLDC Control

**Field-Oriented Control for brushless DC motors on STM32F401RE — with Modbus RTU over RS-485.**

[![Language](https://img.shields.io/badge/C-Embedded-22D3EE?style=for-the-badge&labelColor=0E1626)](https://en.wikipedia.org/wiki/C_(programming_language))
[![MCU](https://img.shields.io/badge/STM32F401RE-Nucleo--64-5A8DFF?style=for-the-badge&labelColor=0E1626)](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
[![Toolchain](https://img.shields.io/badge/arm--none--eabi--gcc-CMake-8A2BE2?style=for-the-badge&labelColor=0E1626)](https://gnu-mcu-eclipse.github.io/toolchain/arm/)
[![License](https://img.shields.io/badge/License-MIT-34D399?style=for-the-badge&labelColor=0E1626)](./LICENSE)

</div>

---

## 🎯 What is it

A bare-metal STM32 firmware that drives a 3-phase BLDC motor with **Field-Oriented Control (FOC)**, reads rotor angle from an **AS5048A** magnetic encoder over SPI, renders status to an **SSD1306** OLED, and accepts commands over **Modbus RTU** through a **MAX485** RS-485 transceiver.

Built on top of the [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) algorithm, ported to STM32 HAL.

---

## ✨ Features

- 🌀 **Field-Oriented Control** of a 3-phase BLDC motor
- 🎛 **PID** velocity / position loops
- 🧭 **AS5048A** magnetic encoder (SPI) for absolute angle feedback
- 📺 **SSD1306** 128×64 OLED (I²C) for live status
- 🔌 **MAX485** RS-485 transceiver with DE/RE direction control — Modbus RTU frame echo on USART1
- 🪛 **USART2** for serial debug / commands
- ⚙️ Built with **STM32CubeMX**-generated HAL and a hand-rolled CMake toolchain file

> Modbus register map is still being expanded — current firmware echoes received frames and exposes a 16-bit command word over USART1.

---

## 🔌 Hardware

| Component | Part        | Bus             |
|-----------|-------------|-----------------|
| 🧠 MCU       | STM32F401RE (Nucleo-F401RE) | —               |
| 🧭 Encoder   | AS5048A     | SPI2            |
| 📺 Display   | SSD1306 OLED 128×64 | I²C1     |
| 🔄 Transceiver | MAX485    | USART1 + DE/RE GPIO |
| 🛠 Debug UART | ST-Link VCP | USART2          |
| ⚡ Driver    | 3-phase BLDC gate driver (TIM1 PWM) | TIM1 CH1/2/3 |
| ⏱ Loop timers | TIM2/3/4   | —               |

Pin assignments live in `stm32_simple-foc.ioc` (open in STM32CubeMX) and the generated `Core/Inc/main.h`.

---

## 🚀 Build & flash

### Option A — CMake + arm-none-eabi-gcc

```bash
git clone https://github.com/Luck-ai/modbus-based-bldc-control
cd modbus-based-bldc-control
mkdir build && cd build
cmake ..
cmake --build .
```

Outputs `stm32_simple-foc.elf` / `.hex` / `.bin` in `build/`. Flash with `st-flash`, `openocd`, or drag-and-drop the `.bin` onto the Nucleo's mass-storage mount.

OpenOCD config for the on-board ST-Link is included: `st_nucleo_f4.cfg`.

### Option B — STM32CubeIDE

1. **File → Open Projects from File System…** and point at the repo root
2. **Project → Build Project**
3. **Run → Debug** (uses ST-Link)

---

## 🗂 Project structure

```text
modbus-based-bldc-control/
├── Core/
│   ├── Inc/                 # headers (main.h, drivers, FOC, PID, encoder, OLED)
│   ├── Src/
│   │   ├── main.c           # peripheral init + control loop
│   │   ├── bldc_driver.c    # PWM / 3-phase output
│   │   ├── bldc_motor.c     # motor object, FOC update
│   │   ├── foc_utils.c      # Park/Clarke, sin/cos LUT
│   │   ├── pid.c            # PID controller
│   │   ├── lpf.c            # low-pass filter
│   │   ├── as5048a.c        # SPI encoder driver
│   │   └── ssd1306*.c       # OLED driver + fonts
│   └── Startup/
├── Drivers/                 # STM32 HAL + CMSIS
├── CMakeLists.txt           # auto-generated from CubeMX template
├── STM32F401RETX_FLASH.ld   # linker script (flash)
├── STM32F401RETX_RAM.ld     # linker script (RAM)
├── st_nucleo_f4.cfg         # OpenOCD config
└── stm32_simple-foc.ioc     # CubeMX project file
```

---

## 🙏 Credits

FOC algorithm ported from **[SimpleFOC](https://github.com/simplefoc/Arduino-FOC)**. STM32 HAL by STMicroelectronics.

---

<div align="center">

**Modbus-Based BLDC Control** · [GitHub](https://github.com/Luck-ai/modbus-based-bldc-control) · [Issues](https://github.com/Luck-ai/modbus-based-bldc-control/issues)

</div>
