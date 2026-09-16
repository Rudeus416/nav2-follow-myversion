#!/bin/bash

# 检查是否以root身份运行
if [ "$(id -u)" != "0" ]; then
    echo "请以root身份运行此脚本。"
    exit 1
fi

# 函数：设置CPU性能模式
set_performance_mode() {
    local governor_dir="$1"
    if [ -w "$governor_dir" ]; then
        echo performance > "$governor_dir/scaling_governor"
        if [ $? -eq 0 ]; then
            echo "CPU性能模式已设置为performance（核心 $2）"
        else
            echo "设置CPU性能模式失败（核心 $2）"
        fi
    else
        echo "无法访问目录：$governor_dir"
    fi
}

# 遍历所有CPU核心并设置性能模式
for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*; do
    if [ -d "$cpu_dir" ]; then
        if [ -d "$cpu_dir/cpufreq" ]; then
            set_performance_mode "$cpu_dir/cpufreq" "${cpu_dir##*/cpu}"
        elif [ -f "$cpu_dir/power/cpuquiet/state" ]; then
            # 对于一些系统可能使用不同的节能机制，这里仅为示例
            echo performance > "$cpu_dir/power/cpuquiet/state"
            echo "CPU性能模式已设置为performance（核心 ${cpu_dir##*/cpu}，通过不同的机制）"
        else
            echo "未找到合适的电源管理接口在 ${cpu_dir##*/cpu}"
        fi
    fi
done

echo "所有支持的CPU核心已尝试设置为性能模式。"
