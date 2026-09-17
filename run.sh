#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RANGER_WS="/home/metaiot/Workspace/RANGER/ranger_ws"
CAMERA_SETUP="/home/metaiot/Workspace/dataCollectToolkit/ros2_driver_ws/install/setup.bash"
ROS_SETUP="/opt/ros/humble/setup.bash"
PYTHON="/home/metaiot/miniconda3/envs/follow_demo/bin/python"
RUNTIME_DIR="${ROOT}/data/runtime"
CONTROL_PID_FILE="${RUNTIME_DIR}/control.pid"
RANGER_PID_FILE="${RUNTIME_DIR}/ranger.pid"
CONTROL_LOG="${RUNTIME_DIR}/control.log"
RANGER_LOG="${RUNTIME_DIR}/ranger.log"
LOCK_FILE="${RUNTIME_DIR}/manager.lock"
STATE_FILE="${RUNTIME_DIR}/state"
CONTROL_PATTERN='^/home/metaiot/miniconda3/envs/follow_demo/bin/python -B (control\.py|/home/metaiot/Workspace/visual_car/control\.py) '
RANGER_LAUNCH_PATTERN='^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch ranger_bringup ranger_mini_v3\.launch\.py'
RANGER_NODE_PATTERN='^/home/metaiot/Workspace/RANGER/ranger_ws/install/ranger_base/lib/ranger_base/ranger_base_node --ros-args'
CAMERA_LAUNCH_PATTERN='^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch hik_camera hik_camera\.launch\.py'
CAMERA_NODE_PATTERN='^/home/metaiot/Workspace/dataCollectToolkit/ros2_driver_ws/install/hik_camera/lib/hik_camera/hik_camera_node --ros-args'

CAN_IF="${CAN_IF:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
WEB_PORT="${WEB_PORT:-8890}"
CAPTURE_FPS="${CAPTURE_FPS:-5}"
CAMERA_INPUT_FPS="${CAMERA_INPUT_FPS:-30}"
PROCESS_FPS="${PROCESS_FPS:-15}"
DEPTH_FPS="${DEPTH_FPS:-5}"
MAX_FRAMES="${MAX_FRAMES:-20}"

mkdir -p "${RUNTIME_DIR}"
exec 9>"${LOCK_FILE}"
flock -w 10 9 || { echo "另一个 run.sh 操作正在执行，请稍后重试。" >&2; exit 1; }

source_ros() {
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  # shellcheck disable=SC1090
  source "${RANGER_WS}/install/setup.bash"
  set -u
}

write_state() {
  printf '%s %s\n' "$1" "$(date --iso-8601=seconds)" >"${STATE_FILE}"
}

read_live_pid() {
  local file="$1" pid=""
  [[ -r "${file}" ]] && read -r pid < "${file}" || true
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    printf '%s\n' "${pid}"
  fi
}

count_processes() {
  local pattern="$1"
  { pgrep -f "${pattern}" 2>/dev/null || true; } | wc -l
}

wait_for_url() {
  local url="$1" tries="${2:-100}"
  for ((i = 0; i < tries; i++)); do
    curl --noproxy '*' -fsS --max-time 1 "${url}" >/dev/null 2>&1 && return 0
    sleep 0.1
  done
  return 1
}

wait_for_exit() {
  local pid="$1"
  for ((i = 0; i < 100; i++)); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.1
  done
  return 1
}

signal_group() {
  local pid="$1" signal="${2:-INT}"
  kill -s "${signal}" -- "-${pid}" 2>/dev/null || kill -s "${signal}" "${pid}" 2>/dev/null || true
}

stop_matching() {
  local pattern="$1" label="$2"
  local -a pids=()
  mapfile -t pids < <(pgrep -f "${pattern}" 2>/dev/null || true)
  ((${#pids[@]} == 0)) && return 0
  echo "正在停止${label}（PID ${pids[*]}）……"
  kill -INT "${pids[@]}" 2>/dev/null || true
  for ((i = 0; i < 100; i++)); do
    mapfile -t pids < <(pgrep -f "${pattern}" 2>/dev/null || true)
    ((${#pids[@]} == 0)) && return 0
    sleep 0.1
  done
  kill -TERM "${pids[@]}" 2>/dev/null || true
  for ((i = 0; i < 50; i++)); do
    mapfile -t pids < <(pgrep -f "${pattern}" 2>/dev/null || true)
    ((${#pids[@]} == 0)) && return 0
    sleep 0.1
  done
  echo "${label}仍未结束：PID ${pids[*]}" >&2
  return 1
}

bringup_can() {
  if ! ip link show "${CAN_IF}" >/dev/null 2>&1; then
    echo "未发现 ${CAN_IF}，请检查 USB-CAN 连接。" >&2
    return 1
  fi

  if ip -details link show "${CAN_IF}" 2>/dev/null \
      | grep -q "state UP" \
      && ip -details link show "${CAN_IF}" 2>/dev/null \
      | grep -q "bitrate ${CAN_BITRATE}"; then
    echo "CAN：${CAN_IF} 已按 ${CAN_BITRATE} bit/s 启用。"
    return 0
  fi

  echo "CAN：正在配置 ${CAN_IF}，sudo 可能要求输入密码。"
  sudo ip link set dev "${CAN_IF}" down 2>/dev/null || true
  sudo ip link set dev "${CAN_IF}" type can bitrate "${CAN_BITRATE}" restart-ms 100
  sudo ip link set dev "${CAN_IF}" up
}

start_services() {
  local control_count ranger_count
  control_count="$(count_processes "${CONTROL_PATTERN}")"
  ranger_count="$(count_processes "${RANGER_LAUNCH_PATTERN}")"
  if ((control_count > 0 || ranger_count > 0)); then
    echo "检测到现有服务（control=${control_count}, ranger=${ranger_count}），为防止重复启动，本次未启动。" >&2
    echo "请先执行：${ROOT}/run.sh stop" >&2
    return 1
  fi

  write_state starting
  bringup_can
  : >"${RANGER_LOG}"
  : >"${CONTROL_LOG}"

  nohup setsid bash -c '
    set -e
    set +u
    source "$1"
    source "$2/install/setup.bash"
    set -u
    exec ros2 launch ranger_bringup ranger_mini_v3.launch.py port_name:="$3"
  ' _ "${ROS_SETUP}" "${RANGER_WS}" "${CAN_IF}" \
    >"${RANGER_LOG}" 2>&1 </dev/null 9>&- &
  local ranger_pid=$!
  printf '%s\n' "${ranger_pid}" >"${RANGER_PID_FILE}"

  source_ros
  local ranger_ready=0
  for ((i = 0; i < 100; i++)); do
    if ! kill -0 "${ranger_pid}" 2>/dev/null; then break; fi
    if ros2 node list 2>/dev/null | grep -qx '/ranger_base_node'; then
      ranger_ready=1
      break
    fi
    sleep 0.1
  done
  if ((ranger_ready == 0)); then
    echo "Ranger 节点启动失败，请查看 ${RANGER_LOG}" >&2
    signal_group "${ranger_pid}" INT
    rm -f "${RANGER_PID_FILE}"
    write_state failed
    return 1
  fi

  nohup setsid bash -c '
    set -e
    set +u
    source "$1"
    source "$2"
    source "$3/install/setup.bash"
    set -u
    cd "$4"
    exec "$5" -B "$4/control.py" \
      --fps "$6" --camera-fps "$7" --process-fps "$8" --depth-fps "$9" \
      --max-frames "${10}" --port "${11}"
  ' _ "${ROS_SETUP}" "${CAMERA_SETUP}" "${RANGER_WS}" "${ROOT}" "${PYTHON}" \
    "${CAPTURE_FPS}" "${CAMERA_INPUT_FPS}" "${PROCESS_FPS}" "${DEPTH_FPS}" \
    "${MAX_FRAMES}" "${WEB_PORT}" \
    >"${CONTROL_LOG}" 2>&1 </dev/null 9>&- &
  local control_pid=$!
  printf '%s\n' "${control_pid}" >"${CONTROL_PID_FILE}"

  if ! wait_for_url "http://127.0.0.1:${WEB_PORT}/api/status" 300; then
    echo "视觉服务启动失败，请查看 ${CONTROL_LOG}" >&2
    signal_group "${control_pid}" INT
    signal_group "${ranger_pid}" INT
    rm -f "${CONTROL_PID_FILE}" "${RANGER_PID_FILE}"
    write_state failed
    return 1
  fi

  # Every managed start requires the operator to explicitly release the stop
  # latch in the web UI before a non-zero command can be emitted.
  curl --noproxy '*' -fsS -X POST \
    "http://127.0.0.1:${WEB_PORT}/api/force-stop" >/dev/null
  write_state running

  echo "启动完成："
  echo "  Ranger PID：${ranger_pid}"
  echo "  视觉控制 PID：${control_pid}"
  echo "  本机页面：http://127.0.0.1:${WEB_PORT}/"
  echo "  局域网页面：http://$(hostname -I | awk '{print $1}'):${WEB_PORT}/"
  echo "  当前已锁存强制停止；确认环境安全后，在页面点击“解除强制停止”。"
  echo "  查看状态：${ROOT}/run.sh status"
  echo "  结束任务：${ROOT}/run.sh stop"
}

stop_services() {
  local control_pid ranger_pid
  write_state stopping
  control_pid="$(read_live_pid "${CONTROL_PID_FILE}")"
  ranger_pid="$(read_live_pid "${RANGER_PID_FILE}")"

  curl --noproxy '*' -fsS --max-time 2 -X POST \
    "http://127.0.0.1:${WEB_PORT}/api/force-stop" >/dev/null 2>&1 || true
  curl --noproxy '*' -fsS --max-time 2 -X POST \
    "http://127.0.0.1:${WEB_PORT}/api/follow/stop" >/dev/null 2>&1 || true

  if [[ -n "${control_pid}" ]]; then
    echo "正在停止视觉控制（PID ${control_pid}）……"
    signal_group "${control_pid}" INT
    wait_for_exit "${control_pid}" || signal_group "${control_pid}" TERM
  fi

  # Handle services started before this manager existed, and orphaned child
  # processes whose pid files were lost.
  stop_matching "${CONTROL_PATTERN}" "遗留视觉服务" || true
  stop_matching "${CAMERA_LAUNCH_PATTERN}" "遗留相机启动进程" || true
  stop_matching "${CAMERA_NODE_PATTERN}" "遗留相机节点" || true

  if pgrep -f '/ranger_base_node --ros-args' >/dev/null 2>&1; then
    echo "正在通过 /cmd_vel 连续发送零速度……"
    timeout 8 "${RANGER_WS}/scripts/stop.bash" >/dev/null 2>&1 || true
  fi

  if [[ -n "${ranger_pid}" ]]; then
    echo "正在停止 Ranger 驱动（PID ${ranger_pid}）……"
    signal_group "${ranger_pid}" INT
    wait_for_exit "${ranger_pid}" || signal_group "${ranger_pid}" TERM
  fi

  stop_matching "${RANGER_LAUNCH_PATTERN}" "遗留 Ranger 启动进程" || true
  stop_matching "${RANGER_NODE_PATTERN}" "遗留 Ranger 节点" || true
  rm -f "${CONTROL_PID_FILE}" "${RANGER_PID_FILE}"
  local remaining_control remaining_ranger
  remaining_control="$(count_processes "${CONTROL_PATTERN}")"
  remaining_ranger="$(count_processes "${RANGER_NODE_PATTERN}")"
  if ((remaining_control == 0 && remaining_ranger == 0)); then
    write_state stopped
    echo "视觉控制和 Ranger 驱动已结束；${CAN_IF} 保持启用。"
  else
    write_state failed
    echo "停止未完成：control=${remaining_control}, ranger_base_node=${remaining_ranger}" >&2
    return 1
  fi
}

show_status() {
  local control_pid ranger_pid control_count ranger_node_count web_ok=0 managed_state="未知"
  control_pid="$(read_live_pid "${CONTROL_PID_FILE}")"
  ranger_pid="$(read_live_pid "${RANGER_PID_FILE}")"
  control_count="$(count_processes "${CONTROL_PATTERN}")"
  ranger_node_count="$(count_processes "${RANGER_NODE_PATTERN}")"
  [[ -r "${STATE_FILE}" ]] && managed_state="$(cat "${STATE_FILE}")"

  echo "进程状态："
  echo "  管理记录：${managed_state}"
  echo "  视觉控制：$([[ -n "${control_pid}" ]] && echo "运行中，PID ${control_pid}" || echo "未由脚本运行")（系统中共 ${control_count} 个）"
  echo "  Ranger：$([[ -n "${ranger_pid}" ]] && echo "运行中，PID ${ranger_pid}" || echo "未由脚本运行")（ranger_base_node 共 ${ranger_node_count} 个）"

  if ip link show "${CAN_IF}" >/dev/null 2>&1; then
    local can_state
    can_state="$(ip -details link show "${CAN_IF}" | sed -n 's/.*can state \([^ ]*\).*/\1/p' | head -n1)"
    echo "  CAN：${CAN_IF}，${can_state:-未知状态}"
  else
    echo "  CAN：未发现 ${CAN_IF}"
  fi

  if payload="$(curl --noproxy '*' -fsS --max-time 2 "http://127.0.0.1:${WEB_PORT}/api/status" 2>/dev/null)"; then
    web_ok=1
    STATUS_PAYLOAD="${payload}" WEB_PORT_VALUE="${WEB_PORT}" python3 - <<'PY'
import json, os
s = json.loads(os.environ["STATUS_PAYLOAD"])
print(f"  网页：正常，http://127.0.0.1:{os.environ.get('WEB_PORT_VALUE', '8890')}/")
print(f"  控制：模式={s['motion_mode']}，跟随={s['following']}，强停={s['force_stopped']}")
print(f"  指令：linear.x={s['linear_x_mps']} m/s，angular.z={s['angular_z_radps']} rad/s")
print(f"  帧率：相机输入={s['tracking_input_fps']}，跟踪={s['tracker_effective_fps']}，深度={s['depth_effective_fps']} FPS")
print(f"  跳帧：最近={s['latest_skipped_camera_frames']}，累计={s['skipped_camera_frames_total']}")
print(f"  同帧融合：累计={s['fused_total']}，最新帧={s['latest_fused_frame_id']}，等待T/M2={s['join_pending_tracking']}/{s['join_pending_depth']}")
PY
  else
    echo "  网页：无法访问"
  fi

  source_ros
  local nodes topic_info
  nodes="$(ros2 node list 2>/dev/null || true)"
  echo "ROS 状态："
  grep -qx '/visual_car_control' <<<"${nodes}" && echo "  /visual_car_control：存在" || echo "  /visual_car_control：不存在"
  grep -qx '/ranger_base_node' <<<"${nodes}" && echo "  /ranger_base_node：存在" || echo "  /ranger_base_node：不存在"
  topic_info="$(ros2 topic info /cmd_vel 2>/dev/null || true)"
  echo "${topic_info}" | sed -n 's/^Publisher count:/  \/cmd_vel 发布者：/p; s/^Subscription count:/  \/cmd_vel 订阅者：/p'

  if [[ -n "${control_pid}" && -n "${ranger_pid}" && "${control_count}" -eq 1 \
        && "${ranger_node_count}" -eq 1 && "${web_ok}" -eq 1 ]]; then
    echo "结论：任务正在正常运行。"
    return 0
  fi
  if [[ "${control_count}" -eq 0 && "${ranger_node_count}" -eq 0 \
        && "${web_ok}" -eq 0 && "${managed_state}" == stopped\ * ]]; then
    echo "结论：任务已通过脚本正常结束。"
    return 0
  fi
  echo "结论：任务未完整运行，或存在重复进程。"
  return 1
}

show_logs() {
  case "${1:-all}" in
    control) tail -n 100 -F "${CONTROL_LOG}" ;;
    ranger) tail -n 100 -F "${RANGER_LOG}" ;;
    all) tail -n 100 -F "${CONTROL_LOG}" "${RANGER_LOG}" ;;
    *) echo "用法：$0 logs [all|control|ranger]" >&2; return 2 ;;
  esac
}

usage() {
  cat <<EOF
用法：
  $0                 启动全部服务（等同于 start）
  $0 start           启动 CAN、Ranger Mini 3.0 和视觉控制
  $0 status          检查进程、网页、ROS、/cmd_vel 和 CAN
  $0 stop            先发送零速度，再结束全部服务
  $0 restart         安全停止后重新启动
  $0 logs [组件]     查看日志；组件可为 all、control、ranger

可选环境变量：CAN_IF、CAN_BITRATE、WEB_PORT、CAPTURE_FPS、CAMERA_INPUT_FPS、PROCESS_FPS、DEPTH_FPS、MAX_FRAMES
EOF
}

case "${1:-start}" in
  start) start_services ;;
  stop) stop_services ;;
  restart) stop_services; start_services ;;
  status) show_status ;;
  logs) show_logs "${2:-all}" ;;
  help|-h|--help) usage ;;
  *) usage >&2; exit 2 ;;
esac
