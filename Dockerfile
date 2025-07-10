FROM ubuntu:22.04
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git gcc-arm-none-eabi gdb-multiarch ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Clone ThreadX and HAL drivers
RUN git clone --depth 1 https://github.com/eclipse-threadx/threadx.git && \
    git clone --depth 1 https://github.com/STMicroelectronics/stm32h7xx_hal_driver.git

# Copy project
COPY . /workspace/project
WORKDIR /workspace/project

CMD ["/bin/bash"]
