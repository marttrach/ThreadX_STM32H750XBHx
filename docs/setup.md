# Setup and Deployment Guide

This guide provides a more detailed walk‑through for building, flashing and running the ThreadX example on an STM32H750XBHx board.

## 1. Build Environment

1. Clone this repository and `cd` into it.
2. Build the Docker image which contains the cross compilation toolchain and required sources:

   ```bash
   docker build -t threadx-h750 .
   ```

3. Run the container and attach a shell:

   ```bash
   docker run --name threadx-h750-docker -it -p 2022:22 threadx-h750 bash
   ```

The container exposes an SSH server on port `2022` for optional remote access. Basic utilities like `vim` and `git` are pre‑installed.

## 2. Building the Firmware

Inside the container invoke the helper script with a CMake preset:

```bash
./build.sh Release   # or Debug
```

The script runs `cmake --preset <preset>` followed by `cmake --build --preset <preset>` and places the result under `build/<preset>/IOT_DUAL_STM32.elf`.

To remove all generated files:

```bash
make clean
```

## 3. Flashing

The project links the image for flash address `0x08000000`. Use **STM32CubeProgrammer** (GUI or CLI) or another SWD tool to program the board:

```bash
STM32_Programmer_CLI -c port=SWD -d build/Release/IOT_DUAL_STM32.elf
```

or just use stm32programmer choose elf to download

## 4. Operation

After reset the application initializes UART5 and prints a greeting. Connect a serial terminal at **115200 8N1** to observe the output and to interact with any future demos.

## 5. Troubleshooting

- Ensure the board is powered and connected via ST‑LINK.
- If the build step fails, verify that the Docker container has network access to fetch dependencies.
- Use `make clean` when switching between presets.

## 6. Roadmap

A high‑level plan for upcoming features is maintained in [roadmap.md](roadmap.md).