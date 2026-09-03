# Ubuntu 24.04 / ROS 2 Jazzy 설치 및 실기 검증

이 문서는 Ubuntu 24.04.4, ROS 2 Jazzy, Orbbec Astra Pro의 UVC RGB 장치에서
직접 확인한 절차입니다. 같은 setup/실행 스크립트는 Humble, Kilted, Lyrical,
Rolling에서도 사용하며 CI가 각 배포판을 별도로 빌드하고 검사합니다.

## 배포판 선택

| Ubuntu | 권장 ROS 2 | `ROS_DISTRO` |
|---|---|---|
| 22.04 | Humble | `humble` |
| 24.04 | Jazzy | `jazzy` |

Kilted, Lyrical, Rolling은 해당 배포판이 설치된 컨테이너나 시스템에서 같은
절차를 사용합니다. 서로 다른 ROS 배포판에서 만든 `build/`, `install/`, `log/`,
`.venv/`를 복사해 사용하지 마세요.

## 1. 시스템 도구 설치

```bash
export ROS_DISTRO=jazzy
sudo apt update
sudo apt install -y \
  python3-venv python3-pip python3-colcon-common-extensions python3-rosdep
rosdep update
```

Humble에서는 `ROS_DISTRO=humble`만 바꿉니다. ROS 설치가 하나뿐이면 프로젝트
스크립트가 자동으로 찾지만, 여러 버전이 있으면 항상 명시하는 편이 안전합니다.

`python3 -m venv`가 `No module named ensurepip`로 실패하면 Ubuntu의
`python3-venv`가 빠진 것입니다. 패키지를 설치하는 것이 권장 방법입니다. 최신
setup은 시스템 `python3-pip`가 있으면 `--without-pip` 방식으로 자동 재시도하고,
이전 실패가 남긴 불완전한 `.venv`도 자동으로 재생성합니다.

## 2. 프로젝트 설치와 빌드

```bash
git clone https://github.com/OIH62/ros2-vision-safety-zones.git
cd ros2-vision-safety-zones
export ROS_DISTRO=jazzy
./setup.sh
```

setup은 Python 의존성을 `.venv`에 설치하고, 반드시 같은 가상환경 Python으로
colcon을 실행합니다. 다음 결과가 저장소의 `.venv/bin/python`을 가리켜야
`torch`와 `ultralytics`가 런타임에도 보입니다.

```bash
head -1 install/person_pose/lib/person_pose/pose_publisher
```

예:

```text
#!/path/to/ros2-vision-safety-zones/.venv/bin/python
```

## 3. Astra Pro RGB로 빠른 확인

공개 저장소에는 Astra/OpenNI depth 드라이버가 포함되지 않습니다. 전체 depth
통합 전에 Astra Pro의 UVC RGB(`/dev/videoN`)만으로 pose와 구역 판정을 확인할
수 있습니다.

장치 번호 확인:

```bash
for device in /dev/video*; do
  echo "$device"
  udevadm info --query=property --name="$device" |
    grep -E 'ID_V4L_PRODUCT|ID_VENDOR_ID|ID_MODEL_ID'
done
```

이 실기 환경에서는 Astra Pro RGB가 `/dev/video2`, USB ID `2bc5:0502`였습니다.
필요한 ROS 도구를 설치하고 RGB 토픽을 발행합니다.

```bash
sudo apt install -y ros-${ROS_DISTRO}-image-tools ros-${ROS_DISTRO}-rviz2
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 run image_tools cam2image --ros-args \
  -p device_id:=2 \
  -p width:=640 \
  -p height:=480 \
  -p reliability:=best_effort \
  -p show_camera:=false \
  -r image:=/camera/color/image_raw
```

`device_id`는 실제 `/dev/videoN`의 N으로 바꿉니다.

## 4. 파이프라인과 RViz2 실행

새 터미널에서:

```bash
cd ros2-vision-safety-zones
export ROS_DISTRO=jazzy
./run_letmc.sh device:=cpu \
  model_path:="$PWD/models/yolov8n-pose.pt"
```

첫 실행은 YOLO pose 모델을 내려받을 수 있습니다. 다른 터미널에서 상태와
RViz2를 함께 실행합니다.

```bash
cd ros2-vision-safety-zones
export ROS_DISTRO=jazzy
./monitor_state.sh
```

`monitor_state.sh`는 RViz2가 있으면
`config/person_zone.rviz`로 `/person_zone/debug_image`를 Best Effort/depth 1로
엽니다. RViz2가 없고 `image_view`만 있으면 이를 대체 뷰어로 사용하며, 둘 다
없으면 텍스트 상태만 표시합니다.

## 5. 정상 여부 확인

```bash
ros2 node list
ros2 topic hz /camera/color/image_raw
ros2 topic hz /person/keypoints
ros2 topic echo /person_pose/diagnostics
ros2 topic echo /person_zone_state
ros2 topic echo /person_zone/diagnostics
```

RGB-only CPU 실기 검증 기준:

```text
/cam2image
/person_pose_publisher
/person_zone_node
/person_zone_debug_node

OK|device=cpu|cpu_fallback=false
```

사람이 없으면 `person_zone_state`는 `NONE`, 사람이 있으면 `LEFT`, `CENTER`,
`RIGHT` 중 하나입니다. depth 드라이버를 실행하지 않은 RGB-only 시험에서
`DEPTH_STALE`은 예상된 진단이며 pose와 2D 구역 판정 실패를 뜻하지 않습니다.

## 문제 해결

### `.venv/bin/activate`가 없음

이전 setup 실패가 만든 불완전한 `.venv`입니다. 최신 코드를 pull하고 다시
실행하면 자동 재생성합니다. `python3-venv`가 없더라도 시스템 `python3-pip`가
설치되어 있으면 setup이 `--without-pip` 방식으로 자동 재시도합니다.

```bash
git pull --ff-only
./setup.sh
```

### `ensurepip` 오류를 무시해도 되는가?

오류만 보고 그대로 진행하면 안 됩니다. 표준 `venv` 생성이 중단되어 불완전한
`.venv`가 남기 때문입니다. 다음 중 하나가 확인되어야 합니다.

- `sudo apt install python3-venv` 후 `./setup.sh`를 다시 실행해 정상 생성한다.
- 최신 `setup.sh`가 `WARN: ensurepip is unavailable; retrying with the system pip...`
  을 출력한 뒤 `Build complete for ROS 2 ...`까지 완료되는지 확인한다.

두 번째 경우에는 `--without-pip`로 환경을 다시 만들고 시스템 `python3-pip`를
통해 가상환경 전용 pip를 설치하므로, 최초의 ensurepip 메시지는 무시해도 됩니다.
이 fallback까지 실패하면 setup은 성공으로 처리하지 않고 설치할 패키지를 안내한
뒤 종료합니다.

### 실행할 때 `No module named torch`

시스템 colcon으로 Python entry point를 만들고 의존성은 `.venv`에 설치했을 때
발생합니다. 최신 `setup.sh`는 `.venv/bin/python -m colcon`으로 빌드하므로 다시
setup하면 됩니다.

```bash
git pull --ff-only
./setup.sh
head -1 install/person_pose/lib/person_pose/pose_publisher
```

### `COLCON_TRACE: unbound variable`

ROS/colcon이 생성한 setup 파일을 `set -u` 상태로 직접 source할 때 발생합니다.
프로젝트의 `run_letmc.sh`, `monitor_state.sh`, `monitor_xyz.sh`는 nounset을 잠시
해제하는 공통 helper를 사용합니다. 최신 버전에서는 직접 우회할 필요가 없습니다.

### RGB는 되지만 `DEPTH_STALE`

Astra/OpenNI depth 드라이버와 `/camera/depth/image_raw`,
`/camera/depth/camera_info`가 없습니다. 카메라 제조사 드라이버를 별도
워크스페이스에 설치한 뒤 실제 토픽명이 다르면 `run_letmc.sh`의 launch 인자로
전달하세요.
