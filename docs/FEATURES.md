# 기능 및 인터페이스

3분할 판정과 hysteresis는 ROS 타입을 사용하지 않는 C++17
`person_zone_core`에 구현되어 있고, `person_zone_node`는 메시지 변환·토픽·Depth
처리를 담당하는 ROS2 어댑터입니다.

## 1. Pose 검출

`person_pose`는 RGB 영상을 입력받아 YOLO Pose를 한 번만 실행하고
`PersonKeypoints`를 발행합니다.

기본 `device: auto`는 CUDA를 사용할 수 있을 때 GPU를 선택하고, CUDA가 없거나
실제 추론이 실패하면 CPU로 전환합니다. 선택 결과는 `/person_pose/device`,
상세 상태는 `/person_pose/diagnostics`에 1 Hz로 발행됩니다.

```text
입력: /camera/color/image_raw
출력: /person/keypoints
```

메시지에는 검출 여부, 원본 영상 크기와 `x`, `y`, `confidence` 배열이
포함됩니다.

## 2. Human Ratio Zone

여러 사람이 검출되면 Pose 노드는 동일한 관절 가중치 기준으로 각 사람의
주 구역을 계산하고 `LEFT > CENTER > RIGHT` 위험도 순으로 한 명을 선택합니다.
같은 구역에 여러 명이 있으면 바운딩 박스가 큰 사람을 우선합니다.

`person_zone_cpp`는 유효한 COCO 17개 관절을 LEFT/CENTER/RIGHT 영역으로
분류하고 부위별 가중치를 합산합니다. 어깨와 골반 등 몸통 관절의 가중치가
가장 큽니다. 검출된 관절의 가중치 합은 항상 100%로 정규화됩니다.

기본 Hysteresis 설정:

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `keypoint_confidence` | `0.25` | 유효 keypoint 최소 confidence |
| `enter_ratio` | `0.60` | 후보 Zone 진입 최소 비율 |
| `keep_ratio` | `0.45` | 현재 Zone 유지 비율 |
| `transition_margin` | `0.15` | 후보와 현재 Zone의 최소 차이 |
| `confirm_frames` | `4` | 상태 전환 확인 프레임 |
| `lost_person_frames` | `20` | 사람 유실 확정 프레임 |

대표점은 Zone 판정에 사용하지 않습니다. 기존 torso median 대표점은
Depth/XYZ와 Exit 위치 추적에만 사용합니다.

## 3. Depth와 XYZ

대표점을 RGB 좌표에서 Depth 좌표로 변환하고 주변 Depth patch의 중앙값으로
거리를 계산합니다. 카메라 내부 파라미터를 사용해 `PointStamped`의 X/Y/Z로
발행합니다.

```text
출력: /person_position
```

## 4. Exit Direction

지원 방향은 `LEFT`, `RIGHT`, `TOP`, `BOTTOM`이며 기본 활성 방향은
`LEFT`입니다.

```yaml
enabled_exit_directions: [LEFT]
exit_left_ratio: 0.15
exit_right_ratio: 0.15
exit_top_ratio: 0.15
exit_bottom_ratio: 0.15
```

모든 방향 활성화:

```yaml
enabled_exit_directions: [LEFT, RIGHT, TOP, BOTTOM]
```

사람이 보이는 동안 마지막 대표점을 저장하고 `lost_person_frames` 이상
사라졌을 때 edge margin을 확인합니다. 한 유실 과정에서 Exit 이벤트는 한
번만 발행됩니다.

## 5. Warning 해제

Exit가 확정되면 `/person_emergency`가 `true`로 유지됩니다.

기본 자동 해제:

```yaml
auto_clear_warning: true
warning_clear_frames: 5
```

사람이 5프레임 연속으로 안정적으로 재검출되면 Warning과 Exit가 초기화됩니다.

선택적 normalized skeleton 재식별:

```yaml
require_same_person_to_clear: true
reid_max_normalized_distance: 0.35
```

수동 ACK:

```bash
ros2 service call /person_warning_ack std_srvs/srv/Trigger {}
```

## 6. Edge, 이벤트 및 이력

- `/person_edge_warning`: 사람이 활성 edge margin에 진입하면 방향 표시
- `/person_exit_event`: 시간, 방향, 마지막 Zone/좌표/XYZ를 JSON으로 발행
- 선택한 CSV 경로: Exit 이벤트 이력

CSV 기본값은 빈 문자열(기록 안 함)입니다. `event_history_path`에 쓰기 가능한
절대 경로를 지정한 배포에서만 기록합니다.

## 7. Diagnostics

`/person_zone/diagnostics`는 다음 상태를 문자열로 발행합니다.

- `OK`
- `POSE_STALE`
- `DEPTH_STALE`
- 복수 문제는 `|`로 연결

Debug 영상 하단에는 다음 정보가 표시됩니다.

```text
RGB Hz | Pose Hz | Debug Hz | 유효 KP | Lost 진행도 | Edge | Diagnostics
```

Hz는 상태를 바꾸지 않는 측정값입니다.

## 8. Debug 영상

`person_zone_debug`의 입력과 출력:

```text
입력:
  /camera/color/image_raw
  /person/keypoints
  /person_zone_state
  /person_position
  /person_exit_direction
  /person_emergency
  /person_edge_warning
  /person_zone/diagnostics

출력:
  /person_zone/debug_image
```

Debug 노드는 YOLO, Depth 또는 Zone 판정을 수행하지 않습니다. 새 RGB
프레임당 최대 한 번 렌더링하며 기본 `debug_fps`는 15 Hz입니다.

하단 상태 바:

```text
ZONE / Warning / Exit | HUMAN RATIO | XYZ
```

## 9. ROS 인터페이스

| 토픽/서비스 | 타입 | 설명 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB 영상 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Depth 영상 |
| `/person/keypoints` | `person_pose_msgs/msg/PersonKeypoints` | Pose 관절 |
| `/person_zone_state` | `std_msgs/msg/String` | Zone 상태 |
| `/person_position` | `geometry_msgs/msg/PointStamped` | 사람 XYZ |
| `/person_exit_direction` | `std_msgs/msg/String` | Exit 방향 |
| `/person_emergency` | `std_msgs/msg/Bool` | Warning latch |
| `/person_edge_warning` | `std_msgs/msg/String` | Edge 사전 경고 |
| `/person_exit_event` | `std_msgs/msg/String` | 상세 JSON 이벤트 |
| `/person_zone/diagnostics` | `std_msgs/msg/String` | stale 진단 |
| `/person_zone/debug_image` | `sensor_msgs/msg/Image` | Debug 영상 |
| `/person_pose/device` | `std_msgs/msg/String` | 실제 YOLO 장치 |
| `/person_pose/diagnostics` | `std_msgs/msg/String` | 장치/fallback 진단 |
| `/person_warning_ack` | `std_srvs/srv/Trigger` | Warning 수동 해제 |
