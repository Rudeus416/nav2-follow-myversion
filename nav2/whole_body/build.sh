#!/usr/bin/env bash
# 【内容标注】用途：自有整车插件构建与注册。
# 对应用户需求：R13 R23（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：声明依赖、构建/安装或插件注册；具体见本文件指令，不改系统 Nav2 源码。
# 本次仅加注释；需求关联不是精确创建/提交记录。
set -eo pipefail
source /opt/ros/humble/setup.bash
plugin_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$plugin_root" -B "$plugin_root/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$plugin_root/install" -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$plugin_root/build" -j2
ctest --test-dir "$plugin_root/build" --output-on-failure
cmake --install "$plugin_root/build"
AMENT_PREFIX_PATH="$plugin_root/install:$AMENT_PREFIX_PATH" LD_LIBRARY_PATH="$plugin_root/install/lib:$LD_LIBRARY_PATH" "$plugin_root/build/plugin_load_test"
# Separate DDS domain and localhost only; this test uses a synthetic grid, no robot topics.
ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/visual-car-whole-body-test AMENT_PREFIX_PATH="$plugin_root/install:$AMENT_PREFIX_PATH" LD_LIBRARY_PATH="$plugin_root/install/lib:$LD_LIBRARY_PATH" "$plugin_root/build/layer_test"
