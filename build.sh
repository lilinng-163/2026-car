#!/bin/bash
# build.sh
# Git Bash 脚本：配置 → 编译 → 烧录 (一键完成)
# 用法: ./build.sh

PRESET="Debug"

# ---- 1. CMake 配置 ----
echo "=== 1/3: CMake Configure (${PRESET}) ==="
cmake --preset "${PRESET}"
if [ $? -ne 0 ]; then
    echo "CMake 配置失败！"
    exit 1
fi

# ---- 2. CMake 编译 ----
echo "=== 2/3: CMake Build (${PRESET}) ==="
cmake --build "./build/${PRESET}"
if [ $? -ne 0 ]; then
    echo "编译失败！"
    exit 1
fi

# ---- 3. 烧录 ----
echo "=== 3/3: Flash ==="
./fire.sh
