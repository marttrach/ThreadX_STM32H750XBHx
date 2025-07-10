# ThreadX_STM32H750XBHx

This repository demonstrates how to build a minimal ThreadX application for the STM32H750XBHx MCU using a Docker based build environment. The example configures UART5 (pins PB12/PB13) and prints `Hello world` once the kernel is running.

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
   docker run -it threadx-h750 --name threadx-h750-docker -p 2022:22 bash
   ```
   The resulting ELF binary will be located in the `build/` directory.

## Source Overview
- `src/main.c` – minimal ThreadX application that initializes UART5 and prints `Hello world`.
- `CMakeLists.txt` – uses the CMake files from ThreadX to build the demo.
- `build.sh` – helper script executed inside the container.

This setup follows the [ThreadX development guidelines](https://github.com/eclipse-threadx/threadx) by building ThreadX as part of the application using CMake and the Arm GNU Toolchain.
