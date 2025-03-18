# Use Ubuntu as the base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    doxygen \
    qtbase5-dev \
    qtchooser \
    qttools5-dev-tools \
    qttools5-dev \
    libboost-all-dev \
    libglib2.0-dev \
    libglibmm-2.4-dev \
    libserialport-dev \
    libusb-1.0-0-dev \
    libftdi1-dev \
    libzip-dev \
    pkg-config \
    python3 \
    python3-pip \
    libudev-dev \
    libqt5svg5-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory (this will be overridden by the volume mount)
WORKDIR /workspace

# Set entrypoint to stay in shell for interactive development
CMD ["/bin/bash"]