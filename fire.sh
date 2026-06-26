#!/bin/bash
# 该脚本使用git bash类linux环境执行，能实现在windows环境下使用openocd烧录程序到stm32f407zgt6

# ---- 配置 ----
OPENOCD_HOME="/d/openocd/xpack-openocd-0.12.0-7-win32-x64/xpack-openocd-0.12.0-7/openocd"

SCRIPTS_DIR="${OPENOCD_HOME}/scripts"
INTERFACE_CFG="${SCRIPTS_DIR}/interface/stlink-v2.cfg"
TARGET_CFG="${SCRIPTS_DIR}/target/stm32f4x.cfg"

FIRMWARE="build/Debug/$(basename "$PWD").bin"

# ---- 1. 检查固件是否存在 ----
if [ ! -f "$FIRMWARE" ]; then
    echo "错误: 固件 $FIRMWARE 不存在，请先编译！"
    exit 1
fi

# ---- 2. 杀掉残留的 openocd 进程（关键！） ----
echo "检查并清理残留的 OpenOCD 进程..."
taskkill //f //im openocd.exe 2>/dev/null
sleep 1

# ---- 3. 烧录（自动重试） ----
echo "烧录 $FIRMWARE ..."

RETRY=0
MAX_RETRY=3
while [ $RETRY -lt $MAX_RETRY ]; do
    RETRY=$((RETRY + 1))
    echo "尝试 $RETRY/$MAX_RETRY ..."

    timeout 60s openocd \
        -f "${INTERFACE_CFG}" \
        -f "${TARGET_CFG}" \
        -c "transport select swd" \
        -c "adapter speed 2000" \
        -c "init" \
        -c "reset halt" \
        -c "flash write_image erase ${FIRMWARE} 0x08000000" \
        -c "verify_image ${FIRMWARE} 0x08000000" \
        -c "reset run" \
        -c "shutdown"

    if [ $? -eq 0 ]; then
        echo "烧录成功！"
        exit 0
    fi

    echo "  失败，清理后重试..."
    taskkill //f //im openocd.exe 2>/dev/null
    sleep 3
done

echo "烧录失败！请检查 ST-Link 连接和硬件。"
exit 1 