# Multi-arch image; we force arm64 so it runs natively on Apple Silicon
# (no x86 emulation via Rosetta/QEMU).
FROM --platform=linux/arm64 mcr.microsoft.com/devcontainers/cpp:debian

# Add any extra tooling you want available in the container here, e.g.:
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends gdb valgrind \
#     && apt-get clean && rm -rf /var/lib/apt/lists/*
