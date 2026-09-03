#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/ros_env.sh
source "$PROJECT_DIR/scripts/ros_env.sh"
source_ros2
source_ros_file "$PROJECT_DIR/install/setup.bash"

echo "=== Person Zone 실행 상태 ==="

echo "[INFO] run_letmc.sh의 디버그 노드를 기다립니다."
for _ in $(seq 1 20); do
  NODE_COUNT="$(
    ros2 node list 2>/dev/null |
      grep -xc "/person_zone_debug_node" || true
  )"
  if [ "$NODE_COUNT" -ge 1 ]; then
    break
  fi
  sleep 0.5
done

if [ "$NODE_COUNT" -eq 1 ]; then
  echo "[OK] 최신 디버그 노드: /person_zone_debug_node"
elif [ "$NODE_COUNT" -gt 1 ]; then
  echo "[ERROR] 디버그 노드가 ${NODE_COUNT}개 실행 중입니다."
  echo "        run_letmc.sh와 monitor_state.sh를 모두 종료 후 다시 실행하세요."
  exit 1
else
  echo "[ERROR] /person_zone_debug_node가 실행 중이 아닙니다."
  echo "        먼저 ./run_letmc.sh를 실행하세요."
  exit 1
fi

echo
echo "=== /person_zone/debug_image Publisher ==="
if ros2 topic info /person_zone/debug_image -v; then
  echo
  echo "[확인] Publisher가 /person_zone_debug_node 하나인지 확인하세요."
else
  echo "[ERROR] /person_zone/debug_image 토픽을 찾을 수 없습니다."
fi

echo
echo "=== 디버그 영상 및 Zone 상태 모니터링 시작 ==="

# 디버그 영상 보기. RViz2가 있으면 저장소의 Best Effort 설정을 사용하고,
# 최소 ROS 설치에서는 image_view가 있을 때만 대체 뷰어로 사용한다.
VIEW_PID=""
if command -v rviz2 >/dev/null 2>&1; then
  rviz2 -d "$PROJECT_DIR/config/person_zone.rviz" &
  VIEW_PID=$!
elif ros2 pkg prefix image_view >/dev/null 2>&1; then
  ros2 run image_view image_view \
    --ros-args \
    -r image:=/person_zone/debug_image &
  VIEW_PID=$!
else
  echo "[WARN] RViz2/image_view가 없어 텍스트 모니터만 시작합니다."
fi

# 이 스크립트가 시작한 프로세스만 종료한다.
cleanup() {
  if [ -n "$VIEW_PID" ]; then
    kill "$VIEW_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# Zone / Warning / Exit 통합 상태 출력
ros2 run person_zone_debug status_monitor
