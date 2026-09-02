# LeTMC-520 화면 3분할 안전 테스트

LeTMC-520(Astra Pro)의 RGB/Depth 영상을 받아 사람의 관절 위치를 기준으로
화면을 `LEFT / CENTER / RIGHT` 세 구역으로 판정하는 ROS2 프로젝트입니다.
카메라 드라이버와 판정 코드를 분리했기 때문에 Humble에 고정되지 않으며,
표준 영상 토픽을 내는 다른 RGB-D 카메라에도 연결할 수 있습니다.

> 이 코드는 안전 보조용 시험 소프트웨어입니다. 인증된 비상정지 장치나
> 안전 PLC를 대신할 수 없습니다.

## 이번 공개본의 핵심

- ROS와 무관한 C++17 `person_zone_core`로 3분할 판정 로직 분리
- Humble/Jazzy/Kilted/Lyrical/Rolling 다중 배포판 CI
- `/opt/ros/humble`, `/home/oih` 같은 고정 경로 제거
- 카메라 토픽·모델·장치·임계값을 launch/YAML로 교체 가능
- `device: auto`: CUDA 사용 가능 시 GPU, 없으면 CPU
- GPU가 실제 추론에서 실패해도 모델을 CPU로 다시 로드해 계속 실행
- 라이선스가 불명확한 카메라 드라이버와 모델 가중치는 공개 저장소에서 제외

## 설치와 실행

```bash
git clone https://github.com/OIH62/letmc-person-zone-ros2.git
cd letmc-person-zone-ros2
export ROS_DISTRO=humble  # ROS2가 여러 버전 설치된 경우만 지정
./setup.sh
```

LeTMC 카메라 드라이버를 먼저 실행한 다음:

```bash
./run_letmc.sh
```

기존 시스템의 토픽명이 다르면 실행할 때 바로 지정합니다.

```bash
./run_letmc.sh \
  color_topic:=/my_camera/color/image_raw \
  depth_topic:=/my_camera/depth/image_raw \
  camera_info_topic:=/my_camera/depth/camera_info
```

GPU/CPU 상태 확인:

```bash
ros2 topic echo /person_pose/device
ros2 topic echo /person_pose/diagnostics
```

통합 방법과 인터페이스는 [통합 가이드](docs/INTEGRATION.md), 기능은
[기능 문서](docs/FEATURES.md), 현장 점검은 [운영 가이드](docs/RUNBOOK.md)를
참조하세요.
