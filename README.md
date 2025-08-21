# ThreadX_STM32H750XBHx

This repository demonstrates how to build a minimal ThreadX application for the STM32H750XBHx MCU using a Docker based build environment.

## Prerequisites
- Docker installed on the host machine

## Build Steps
The provided `Dockerfile` sets up an Ubuntu container with the Arm GNU toolchain, CMake and Ninja. It also clones the official ThreadX sources, the STM32H7 HAL drivers and the CMSIS device headers. Newlib is installed so `nosys.specs` is available. Basic utilities like SSH and vim are installed, and a default RSA key pair is generated so the container is ready for SSH connections.

1. Build the Docker image:
   ```bash
   docker build -t threadx-h750 .
   ```
2. Run the container and build the example inside:
   ```bash
   docker run --name threadx-h750-docker -it -p 2022:22 threadx-h750 bash
   ```
   Inside the container run `./build.sh` with a preset name to configure and
   build the project in a single step. For example:
   ```bash
   ./build.sh Release
   ```
   The generated ELF file will be located under `build/Release/NEW_STM32.elf`.

3. To remove generated build files run:
   ```bash
   make clean
   ```

## Flashing the application

The project now includes a startup file and linker script so the
application is linked for flash address `0x08000000`. Use STM32CubeProgrammer
or a similar tool to load `build/Release/IOT_DUAL_STM32.elf` onto the board.

## Source Overview
- `src/main.c` – minimal ThreadX application that initializes UART5 and prints `Hello world`.
- `CMakeLists.txt` – uses the CMake files from ThreadX to build the demo.
- `build.sh` – helper script executed inside the container.

This setup follows the [ThreadX development guidelines](https://github.com/eclipse-threadx/threadx) by building ThreadX as part of the application using CMake and the Arm GNU Toolchain.

## Advanced Communication Hub Example

For a conceptual design of turning the STM32H750XBHx into an I2C-based communication hub that controls other peripherals (UART, SPI, GPIO, PWM, ADC, CAN) using ThreadX message queues, see [docs/communication_hub.md](docs/communication_hub.md).
This document links to a small C skeleton showing how the threads and queues fit together.
