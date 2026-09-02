# 실행 및 운영 가이드

## 시작 순서

카메라 드라이버를 먼저 실행해 RGB 토픽이 발행되는지 확인합니다.

```bash
ros2 topic hz /camera/color/image_raw
```

프로젝트 디렉터리에서 파이프라인을 실행합니다.

```bash
./run_letmc.sh
```

새 터미널에서 상태와 화면을 확인합니다.

```bash
./monitor_state.sh
```

`monitor_state.sh`는 추론 노드를 새로 만들지 않습니다. 이미 실행 중인 debug
노드가 하나인지 확인하고 viewer와 텍스트 모니터만 시작합니다.

## ROS2 버전 선택

ROS2가 하나만 설치돼 있으면 스크립트가 자동으로 찾습니다. 여러 버전이
설치된 경우 같은 터미널에서 원하는 배포판을 지정합니다.

```bash
export ROS_DISTRO=jazzy
./run_letmc.sh
```

다른 배포판에서 사용할 때는 `build/`, `install/`, `log/`를 복사하지 말고 해당
환경에서 다시 `./setup.sh`를 실행해야 합니다.

## GPU/CPU 확인

```bash
ros2 topic echo /person_pose/device
ros2 topic echo /person_pose/diagnostics
```

정상 예:

```text
data: cuda:0
data: OK|device=cuda:0|cpu_fallback=false
```

GPU 초기화 또는 추론이 실패해 CPU로 전환된 예:

```text
data: cpu
data: GPU_INFERENCE_FAILED: ...|device=cpu|cpu_fallback=true
```

CPU로 강제 실행:

```bash
./run_letmc.sh device:=cpu
```

CPU 전환 후 부하가 높으면 `cpu_process_fps`와 `image_size`를 낮춥니다.

## 종료 및 재시작

코드를 빌드해도 실행 중인 프로세스에는 자동 반영되지 않습니다.

1. monitor 터미널에서 `Ctrl+C`
2. pipeline 터미널에서 `Ctrl+C`
3. `./setup.sh` 또는 `colcon build --symlink-install`
4. 카메라와 pipeline 재실행

## 중복 노드와 성능 확인

```bash
ros2 node list
ros2 topic info /person_zone/debug_image -v
ros2 topic hz /camera/color/image_raw
ros2 topic hz /person/keypoints
ros2 topic hz /person_zone/debug_image
```

정상 상태에서는 `/person_zone_debug_node`와 debug image publisher가 각각
하나입니다. RGB는 정상이고 Pose만 느리면 YOLO 장치와 FPS를 확인하고, Pose는
정상인데 Debug만 느리면 viewer 또는 debug 렌더링 부하를 확인합니다.

## QoS

RGB, depth, keypoints 입력은 SensorDataQoS(Best Effort, Keep Last)입니다.
RViz2에서 debug image를 볼 때도 Best Effort/depth 1을 권장합니다.

## Conda Python과 ROS Python 충돌

`rosidl_adapter`, `em`, `catkin_pkg`, Python ABI 오류에 Conda 경로가 표시되면
`./setup.sh`로 다시 빌드합니다. 설치 스크립트는 기본적으로 ROS deb 설치가
사용하는 `/usr/bin/python3` 기반 가상환경을 만듭니다. ROS를 다른 Python으로
설치했다면 다음처럼 지정합니다.

```bash
ROS_PYTHON_EXECUTABLE=/path/to/ros/python ./setup.sh
```

## 카메라 창이 뜨지 않을 때

```bash
ros2 topic info /camera/color/image_raw -v
ros2 topic info /person_zone/debug_image -v
echo "$DISPLAY"
```

카메라 토픽명이 다르면 launch 인자로 연결합니다. 예:

```bash
./run_letmc.sh color_topic:=/front/color/image_raw
```

## Warning 수동 해제

```bash
ros2 service call /person_warning_ack std_srvs/srv/Trigger '{}'
```

## 이벤트 CSV

기본값은 파일 기록 꺼짐입니다. 필요한 배포에서만 쓰기 가능한 절대 경로를
전달합니다.

```bash
./run_letmc.sh event_history_path:=/var/tmp/person_exit_events.csv
```
