# Multi-arch image; we force arm64 so it runs natively on Apple Silicon
# (no x86 emulation via Rosetta/QEMU).
FROM --platform=linux/arm64 mcr.microsoft.com/devcontainers/cpp:debian

# Linting and formatting — install clang 23 from the official LLVM apt repo
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends lsb-release wget gnupg software-properties-common \
    && wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc \
    && add-apt-repository "deb http://apt.llvm.org/$(lsb_release -cs)/ llvm-toolchain-$(lsb_release -cs)-23 main" \
    && apt-get update \
    && apt-get -y install --no-install-recommends clang-format-23 clang-tidy-23 \
    && ln -s /usr/bin/clang-format-23 /usr/local/bin/clang-format \
    && ln -s /usr/bin/clang-tidy-23   /usr/local/bin/clang-tidy \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install CMake 4.3.3 from Kitware's pre-built aarch64 binary
RUN curl -fsSL https://github.com/Kitware/CMake/releases/download/v4.3.3/cmake-4.3.3-linux-aarch64.sh \
      -o /tmp/cmake.sh \
 && sh /tmp/cmake.sh --prefix=/usr/local --skip-license \
 && rm /tmp/cmake.sh

# Install mcap CLI
RUN curl -fsSL https://github.com/foxglove/mcap/releases/latest/download/mcap-linux-arm64 \
      -o /usr/local/bin/mcap \
    && chmod +x /usr/local/bin/mcap
