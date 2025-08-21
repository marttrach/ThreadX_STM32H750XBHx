# I2C Communication Hub Example for STM32H750XBHx

This document sketches an approach for turning the STM32H750XBHx into an I/O communication hub accessible via I2C. The microcontroller acts as an I2C slave with address `0x36` (decimal `54`) and interprets incoming messages that request operations on other peripherals (UART, SPI, GPIO, PWM, ADC, CAN). Results are returned to the I2C master using a simple structured format such as JSON or MessagePack.

## Overview
1. **I2C Slave Interface**
   - Configure `I2C2` (or another instance) in slave mode.
   - Set the own address to `0x36`.
   - Incoming frames are parsed into commands specifying which peripheral to access and what operation to perform.

2. **ThreadX Tasks**
   - Create a dedicated ThreadX thread for I2C handling. This thread receives commands from the master and posts them to a FIFO message queue.
   - Spawn separate threads for each peripheral type (UART, SPI, GPIO, PWM, ADC, CAN). These threads wait on the queue for commands addressed to them.
   - Each peripheral thread performs the requested operation and enqueues the response payload for the I2C thread to transmit back to the master.

3. **Message Format**
   - Use a lightweight structured format. MessagePack (`umsgpack`) works well, though simple JSON is also possible.
   - A command might contain fields such as `"target"` (e.g. `"UART"`, `"SPI"`), `"operation"` (read/write), and `"data"`.
   - Responses mirror the same structure and can contain a status field and any returned data.

4. **FIFO and Synchronization**
   - A single ThreadX queue (or multiple queues per peripheral) ensures commands are processed in order and prevents data loss.
   - Mutexes or semaphores guard shared resources if needed (for example, if multiple commands attempt to access the same SPI bus).

## Example Flow
1. I2C master sends a packet requesting a UART transmission.
2. The I2C thread parses the packet and posts a message to the UART thread's queue.
3. The UART thread sends the data out over UART and posts a completion response back to the I2C thread.
4. The I2C thread packages the response (e.g., `{ "status": "ok" }`) and writes it onto the I2C bus for the master to read.

Using ThreadX threads and message queues keeps each peripheral handler independent while avoiding contention on shared resources. Expanding the command set allows the hub to expose many MCU features over a single I2C interface.

## Example Command Format

A simple JSON request from the master might resemble:

```json
{ "target": "GPIO", "operation": "read", "pin": 7 }
```

The corresponding response would be:

```json
{ "status": "ok", "data": 1 }
```

## Pseudocode Outline

Below is a very small sketch of how the threads could be organized:

```c
void i2c_thread(void *arg) {
    for (;;) {
        command_t cmd = read_from_i2c();
        tx_queue_send(&command_queue, &cmd, TX_WAIT_FOREVER);
    }
}

void peripheral_thread(void *arg) {
    for (;;) {
        command_t cmd;
        tx_queue_receive(&command_queue, &cmd, TX_WAIT_FOREVER);
        response_t rsp = handle_command(&cmd);
        tx_queue_send(&response_queue, &rsp, TX_WAIT_FOREVER);
    }
}
```

The I2C thread would also be responsible for sending `response_t` objects back to the master.

## Reference Implementation

A minimal C skeleton showing the ThreadX threads and queues is provided in [src/i2c_hub.c](../src/i2c_hub.c). It illustrates how an I2C thread forwards commands to peripheral threads using `TX_QUEUE`.
