# Multi-arch image; we force arm64 so it runs natively on Apple Silicon
# (no x86 emulation via Rosetta/QEMU).
FROM --platform=linux/arm64 mcr.microsoft.com/devcontainers/cpp:debian

# Toolchain: install clang 22 (newest LLVM release; clang 23 is not released yet)
# from the official LLVM apt repo. The repo is added by hand because trixie has
# no `software-properties-common`/`add-apt-repository`.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends ca-certificates wget gnupg \
    && wget -qO /etc/apt/trusted.gpg.d/apt.llvm.org.asc https://apt.llvm.org/llvm-snapshot.gpg.key \
    && echo "deb http://apt.llvm.org/trixie/ llvm-toolchain-trixie-22 main" > /etc/apt/sources.list.d/llvm.list \
    && apt-get update \
    && apt-get -y install --no-install-recommends clang-22 clang-format-22 clang-tidy-22 \
    && ln -sf /usr/bin/clang++-22     /usr/local/bin/clang++-22 \
    && ln -sf /usr/bin/clang-format-22 /usr/local/bin/clang-format \
    && ln -sf /usr/bin/clang-tidy-22   /usr/local/bin/clang-tidy \
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
