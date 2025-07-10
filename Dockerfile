FROM ubuntu:22.04
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git gcc-arm-none-eabi gdb-multiarch ca-certificates \
    openssh-client openssh-server vim \
    && rm -rf /var/lib/apt/lists/*

# Prepare SSH keys so the container can use SSH immediately
RUN ssh-keygen -A && \
    mkdir -p /root/.ssh && \
    ssh-keygen -t rsa -f /root/.ssh/id_rsa -N "" -q

WORKDIR /workspace

# Clone ThreadX and HAL drivers
RUN git clone --depth 1 https://github.com/eclipse-threadx/threadx.git && \
    git clone --depth 1 https://github.com/STMicroelectronics/stm32h7xx_hal_driver.git

# Copy project Directly Build and Test Can Use ./build.sh
COPY . /workspace/project
WORKDIR /workspace/project

CMD ["/bin/bash"]
