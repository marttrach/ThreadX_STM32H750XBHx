FROM ubuntu:22.04
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git gcc-arm-none-eabi gdb-multiarch \
    libnewlib-arm-none-eabi libnewlib-dev libstdc++-arm-none-eabi-newlib \
    libstdc++-arm-none-eabi-dev ca-certificates \
    openssh-client openssh-server vim \
    && rm -rf /var/lib/apt/lists/*

# Prepare SSH keys so the container can use SSH immediately
RUN ssh-keygen -A && \
    mkdir -p /root/.ssh && \
    ssh-keygen -t rsa -f /root/.ssh/id_rsa -N "" -q

WORKDIR /workspace

# Clone ThreadX, HAL drivers and CMSIS device headers
# RUN git clone --depth 1 https://github.com/STMicroelectronics/stm32-mw-threadx.git && \
#     git clone --depth 1 https://github.com/STMicroelectronics/stm32h7xx_hal_driver.git && \
#     git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_h7.git && \
#     git clone --depth 1 https://github.com/STMicroelectronics/STM32CubeH7.git && \
#     git clone --depth 1 https://github.com/STMicroelectronics/stm32-mw-netxduo.git && \
#     git clone --depth 1 https://github.com/STMicroelectronics/stm32-mw-usbx.git && \
#     git clone --depth 1 https://github.com/STMicroelectronics/stm32-mw-filex.git && \
#     git clone --depth 1 https://github.com/STMicroelectronics/stm32-is42s32800g.git

# Copy project Directly Build and Test Can Use ./build.sh
COPY . /workspace/project
RUN cd /workspace/project && \
    git submodule update --init               \
    && echo "[DONE] git submodule update --init"
WORKDIR /workspace/project

CMD ["/bin/bash"]
