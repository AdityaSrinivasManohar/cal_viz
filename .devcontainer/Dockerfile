# Multi-arch image; we force arm64 so it runs natively on Apple Silicon
# (no x86 emulation via Rosetta/QEMU).
FROM --platform=linux/arm64 mcr.microsoft.com/devcontainers/cpp:debian

# Add any extra tooling you want available in the container here, e.g.:
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends gdb valgrind \
#     && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install CMake 4.3.3 from Kitware's pre-built aarch64 binary
RUN curl -fsSL https://github.com/Kitware/CMake/releases/download/v4.3.3/cmake-4.3.3-linux-aarch64.sh \
      -o /tmp/cmake.sh \
 && sh /tmp/cmake.sh --prefix=/usr/local --skip-license \
 && rm /tmp/cmake.sh

# Install mcap CLI
RUN curl -fsSL https://github.com/foxglove/mcap/releases/latest/download/mcap-linux-arm64 \
      -o /usr/local/bin/mcap \
    && chmod +x /usr/local/bin/mcap
