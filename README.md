# Step 3: AGV 3D 맵핑 및 환경 인식

## 1. 프로젝트 개요

ROS2와 Gazebo 환경에서 AGV에 탑재된 3D LiDAR 센서 데이터를 활용하여 실시간 3D 맵을 생성합니다. `OctoMap`을 이용한 3D 점유 격자 지도 생성을 통해 로봇의 3D 환경 인식 능력을 구현합니다.

## 2. 목표

- **Gazebo**: 3D LiDAR를 활용하여 시뮬레이션 내의 환경을 스캔합니다.
- **3D Mapping**: `OctoMap`을 활용하여 실시간으로 3D 점유 격자 지도를 생성하고 RViz에서 시각화합니다.
- **시각화 파이프라인**: 센서 데이터가 3D 지도로 변환되는 과정을 시각적으로 확인 가능한 파이프라인을 구축합니다.

## 3. 결과물 (Artifacts)

### 시스템 구성도

![시스템 구성도](https://github.com/user-attachments/assets/c142734d-facc-47b1-a3cf-0b1366612dc1)

### 실행 화면

![3D 맵핑 실행 화면 1](https://github.com/user-attachments/assets/dc0c07c2-bca1-4067-88ee-adc443aa75f7)

## 4. 핵심 패키지

- `agv_pro_gazebo`: 3D LiDAR가 장착된 AGV 모델과 시뮬레이션 환경을 로드합니다.
- `libgazebo_ros_ray_sensor`: 3D LiDAR가 시뮬레이션 환경을 스캔하고 포인트 클라우드를 발행합니다.
- `octomap_server`: `libgazebo_ros_ray_sensor`로부터 생성된 포인트 클라우드 데이터를 입력받아 OctoMap을 생성합니다.

## 5. 실행 방법

### Gazebo 실행 및 OctoMap 생성

```bash
# 워크스페이스 환경 설정
source ~/step3/install/setup.bash
# Gazebo와 OctoMap을 함께 실행하는 통합 런치 파일 실행
ros2 launch agv_pro_gazebo agv_pro_gazebo.launch.py 
```
> 실행 후 RViz에서 `OccpancyGrid`와 `octomap_binary` / `octomap_binary` 토픽을 추가하여 3D 맵과 OctoMap이 생성되는 과정을 확인할 수 있습니다.

## 6. 자체 평가 및 개선 방향

### 미비한 점

- **맵 활용의 한계**: 생성된 3D 지도(OctoMap)가 시각화에만 사용되고, 자율 주행의 충돌 회피에는 직접적으로 연동되지 않았습니다.
- **성능 문제**: 대규모 환경에서 3D 맵을 실시간으로 처리할 때 계산 부하가 커져 성능 저하가 발생할 수 있습니다.

### 개선 방향

- **3D 충돌 회피 적용**: 생성된 OctoMap을 Nav2의 Costmap 레이어 플러그인으로 통합하여, 2D 지도에 없는 3D 장애물(예: 공중에 매달린 장애물)을 회피하는 3D 네비게이션을 구현합니다.
- **성능 최적화**: RTAB-Map의 파라미터(특징점 검출, 루프 클로저 등)와 OctoMap의 해상도를 조절하여 실시간성과 맵의 정밀도 사이의 균형을 최적화합니다.
