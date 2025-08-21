# I2C Communication Hub Example for STM32H750XBHx

This document sketches an approach for turning the STM32H750XBHx into an I/O communication hub accessible via I2C. The microcontroller acts as an I2C slave with address `0x36` (decimal `54`) and interprets incoming messages that request operations on other peripherals (UART, SPI, GPIO, PWM, ADC, CAN). Results are returned to the I2C master using a simple structured format such as JSON or MessagePack.

> **Note:** For instructions on building and flashing the base firmware see [setup.md](setup.md).

## Components
The hub logic is split across several source files:
- `Src/i2c_hub.c` – core dispatcher and DMA‑based I2C callbacks.
- `Src/i2c_hub_uart.c` – forwards commands to UART handlers.
- `Src/i2c_hub_mem.c` – exposes SDRAM for memory reads and writes.
- `Src/i2c_hub_filex.c` – example hooks to the FileX filesystem.
- `Src/i2c_hub_w55k.c` – networking prototype using a WIZnet W55K.
- `Inc/i2c_hub*.h` – public message formats and queue definitions.

Each module communicates through `TX_QUEUE` objects and optional mutexes for shared resources.

## Detailed Flow
1. The I2C interrupt callback receives a frame and posts a `hub_cmd_t` message to `cmd_queue`.
2. A worker thread parses the command and dispatches it to the appropriate peripheral queue.
3. When a peripheral completes the request it enqueues a `hub_rsp_t` on `rsp_queue`.
4. The I2C thread packages the response and transmits it back to the master. A busy frame is sent if no response is ready.

The queues and worker thread are defined near the top of `Src/i2c_hub.c` and show how ThreadX primitives coordinate the transfers.

## Message Format
A lightweight structured format is recommended. MessagePack (`umsgpack`) works well, though simple JSON is also possible. A command might contain fields such as `"target"` (e.g. `"UART"`, `"SPI"`), `"operation"` (read/write) and `"data"`. Responses mirror the same structure and can contain a status field and any returned data.

## Extending the Hub
To support another peripheral:
1. Create a `<peripheral>_thread` implementation and its header file.
2. Define command/response structures.
3. Add a queue for the peripheral and register it in `i2c_hub.c`.
4. Update the command parser to route messages to the new queue.

Refer to `Src/i2c_hub_uart.c` for a compact example.

## Roadmap
- Define a binary protocol and versioning scheme.
- Implement DMA‑based transfers for large payloads.
- Add security (command authentication and CRC checks).
- Provide high level host libraries for easier integration.

For upcoming enhancements and long‑term plans refer to the project [roadmap](roadmap.md).
