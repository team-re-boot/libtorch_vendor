FROM hakuturu583/cuda_ros:lt4-humble-cuda-tensorrt-12.2.12-devel as build_stage
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
    python3-vcstool git python3-colcon-common-extensions python3-rosdep ccache\
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV USE_CCACHE 1
ENV PATH "/usr/lib/ccache:${PATH}"
ENV CC '/usr/lib/ccache/gcc'
ENV CXX '/usr/lib/ccache/g++'
ENV CCACHE_DIR "$HOME/.cache/ccache/"
RUN mkdir -p ~/.cache/ccache
RUN echo "max_size = 20G" >> ~/.cache/ccache/ccache.conf
RUN pip3 install cmake --upgrade

RUN mkdir -p libtorch_ws/src/libtorch_vendor
WORKDIR libtorch_ws/src/libtorch_vendor
ADD . .
WORKDIR ../../
ENV USE_NCCL 0
ENV USE_DISTRIBUTED 1
ENV TORCH_CUDA_ARCH_LIST 8.7
RUN mkdir /base_packages
RUN rosdep init && rosdep update
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -iry --from-paths src && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
RUN --mount=type=cache,target=~/.cache/ccache source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --install-base /base_packages
