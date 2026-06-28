FROM mcr.microsoft.com/devcontainers/cpp:debian
ARG TARGETARCH

# Toolchain: install clang 22 (newest LLVM release; clang 23 is not released yet)
# from the official LLVM apt repo. The repo is added by hand because trixie has
# no `software-properties-common`/`add-apt-repository`.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends ca-certificates wget gnupg \
    && wget -qO /etc/apt/trusted.gpg.d/apt.llvm.org.asc https://apt.llvm.org/llvm-snapshot.gpg.key \
    && echo "deb http://apt.llvm.org/trixie/ llvm-toolchain-trixie-22 main" > /etc/apt/sources.list.d/llvm.list \
    && apt-get update \
    && apt-get -y install --no-install-recommends clang-22 clang-format-22 clang-tidy-22 clangd-22 \
    && ln -sf /usr/bin/clang++-22     /usr/local/bin/clang++-22 \
    && ln -sf /usr/bin/clang-format-22 /usr/local/bin/clang-format \
    && ln -sf /usr/bin/clang-tidy-22   /usr/local/bin/clang-tidy \
    && ln -sf /usr/bin/clangd-22       /usr/local/bin/clangd \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install CMake 4.3.3 from Kitware's pre-built binary.
# CMake uses "aarch64"/"x86_64"; Docker TARGETARCH uses "arm64"/"amd64".
RUN CMAKE_ARCH=$([ "$TARGETARCH" = "arm64" ] && echo "aarch64" || echo "x86_64") \
 && curl -fsSL https://github.com/Kitware/CMake/releases/download/v4.3.3/cmake-4.3.3-linux-${CMAKE_ARCH}.sh \
      -o /tmp/cmake.sh \
 && sh /tmp/cmake.sh --prefix=/usr/local --skip-license \
 && rm /tmp/cmake.sh

# Install mcap CLI (TARGETARCH matches mcap's naming: arm64/amd64)
RUN curl -fsSL https://github.com/foxglove/mcap/releases/latest/download/mcap-linux-${TARGETARCH} \
      -o /usr/local/bin/mcap \
    && chmod +x /usr/local/bin/mcap
