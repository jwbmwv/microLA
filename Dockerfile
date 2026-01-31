# Multi-stage Dockerfile for MicroLA development and testing
# Provides consistent cross-platform build environment

# Stage 1: Base development environment
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    python3 \
    python3-pip \
    clang-15 \
    clang-tidy-15 \
    clang-format-15 \
    gcc-arm-none-eabi \
    qemu-system-arm \
    lcov \
    gcovr \
    valgrind \
    cppcheck \
    && rm -rf /var/lib/apt/lists/*

# Install Google Test
RUN cd /tmp && \
    git clone --depth 1 --branch v1.14.0 https://github.com/google/googletest.git && \
    cd googletest && \
    cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build && \
    cmake --install build && \
    cd .. && rm -rf googletest

# Install ARM GCC toolchain
RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 -O /tmp/gcc-arm.tar.bz2 && \
    tar -xjf /tmp/gcc-arm.tar.bz2 -C /opt && \
    rm /tmp/gcc-arm.tar.bz2

ENV PATH="/opt/gcc-arm-none-eabi-10.3-2021.10/bin:${PATH}"

WORKDIR /workspace

# Stage 2: Development environment with tools
FROM base AS dev

# Install additional development tools
RUN pip3 install --no-cache-dir \
    conan==2.0.0 \
    cmake-format \
    pre-commit

# Install Zephyr SDK (for embedded testing)
RUN wget -q https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.1/zephyr-sdk-0.16.1_linux-x86_64.tar.xz && \
    tar -xf zephyr-sdk-0.16.1_linux-x86_64.tar.xz -C /opt && \
    /opt/zephyr-sdk-0.16.1/setup.sh -t all -h -c && \
    rm zephyr-sdk-0.16.1_linux-x86_64.tar.xz

ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-0.16.1

# Stage 3: Testing environment
FROM base AS test

COPY . /workspace/microla
WORKDIR /workspace/microla

# Build and run tests
RUN cmake --preset debug && \
    cmake --build --preset debug && \
    ctest --preset debug

# Stage 4: Coverage analysis
FROM test AS coverage

RUN cmake --preset coverage && \
    cmake --build --preset coverage && \
    ctest --preset coverage && \
    lcov --capture --directory . --output-file coverage.info && \
    lcov --remove coverage.info '/usr/*' --output-file coverage.info && \
    lcov --list coverage.info

# Stage 5: Benchmarking
FROM base AS benchmark

COPY . /workspace/microla
WORKDIR /workspace/microla

RUN cmake --preset benchmark && \
    cmake --build --preset benchmark

# Stage 6: Documentation generation
FROM base AS docs

RUN apt-get update && apt-get install -y \
    doxygen \
    graphviz \
    plantuml \
    && rm -rf /var/lib/apt/lists/*

COPY . /workspace/microla
WORKDIR /workspace/microla

RUN doxygen Doxyfile

# Stage 7: Final lightweight image for CI
FROM ubuntu:22.04 AS ci

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["/bin/bash"]

# Usage examples:
#
# Build base image:
#   docker build --target base -t microla:base .
#
# Development environment:
#   docker build --target dev -t microla:dev .
#   docker run -it -v $(pwd):/workspace microla:dev
#
# Run tests:
#   docker build --target test -t microla:test .
#
# Generate coverage:
#   docker build --target coverage -t microla:coverage .
#
# Run benchmarks:
#   docker build --target benchmark -t microla:benchmark .
#   docker run microla:benchmark ./build/benchmark/bench_matrix_multiply
#
# Build documentation:
#   docker build --target docs -t microla:docs .
#   docker run -v $(pwd)/docs:/workspace/microla/docs microla:docs
