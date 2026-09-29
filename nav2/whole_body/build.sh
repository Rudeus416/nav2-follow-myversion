#!/usr/bin/env bash
# 【内容标注 / R13 R23 R41】自有整车导航插件：构建、离线验证和本地安装。
# R41 用户“怎么修正”：隔离 Conda 构建依赖，修复 GLIBCXX_3.4.30 缺失。
# 只改变本脚本的子进程环境；不改系统库、原视觉环境或外部 Nav2 源码。
set -eo pipefail
plugin_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Conda 的 fmt/spdlog 会把旧 libstdc++ 路径写入 ELF RUNPATH。仅退出
# Conda 不会修复已有 CMake 缓存，因此使用干净子进程并重建配置缓存。
# HOME 仅沿用原值；不加载用户 shell 配置，也不继承 LD_PRELOAD 等覆盖项。
exec /usr/bin/env -i HOME="$HOME" LANG=C.UTF-8 PYTHONNOUSERSITE=1 PATH=/usr/bin:/bin:/usr/sbin:/sbin \
    /bin/bash --noprofile --norc -s -- "$plugin_root" <<'NAV2_BUILD'
set -eo pipefail
plugin_root="$1"
source /opt/ros/humble/setup.bash
mkdir -p "$plugin_root/build"
# 防止两个本脚本同时重写同一缓存；锁文件位于自己的生成目录。
exec 9>"$plugin_root/build/.nav2-build.lock"
/usr/bin/flock -n 9 || { echo '整车导航插件正在构建，请等待当前构建结束。' >&2; exit 1; }

# 只重建 CMake 的生成元数据，保留源文件、历史测试日志及旧安装产物。
rm -f -- "$plugin_root/build/CMakeCache.txt"
rm -rf -- "$plugin_root/build/CMakeFiles"
triplet="$(/usr/bin/c++ -dumpmachine)"
system_cmake="/usr/lib/$triplet/cmake"
/usr/bin/cmake -S "$plugin_root" -B "$plugin_root/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$plugin_root/install" \
    -DCMAKE_C_COMPILER=/usr/bin/cc -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF \
    -Dfmt_DIR="$system_cmake/fmt" -Dspdlog_DIR="$system_cmake/spdlog"
/usr/bin/cmake --build "$plugin_root/build" -j2
# 几何回归通过才安装；安装后仍须完成插件加载和图层验证。
/usr/bin/ctest --test-dir "$plugin_root/build" --output-on-failure
/usr/bin/cmake --install "$plugin_root/build"
AMENT_PREFIX_PATH="$plugin_root/install:$AMENT_PREFIX_PATH" \
LD_LIBRARY_PATH="$plugin_root/install/lib:$LD_LIBRARY_PATH" \
    "$plugin_root/build/plugin_load_test"
# 独立 DDS 域、仅本机、合成地图；不连接原导航或发布底盘速度。
ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/visual-car-whole-body-test \
AMENT_PREFIX_PATH="$plugin_root/install:$AMENT_PREFIX_PATH" \
LD_LIBRARY_PATH="$plugin_root/install/lib:$LD_LIBRARY_PATH" \
    "$plugin_root/build/layer_test"
echo '整车导航插件构建、测试、安装完成；未启动小车。'
NAV2_BUILD
