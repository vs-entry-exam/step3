# Step 3 – Sensor & Mapping Visualization

### Project Goal

ROS2에서 수집한 센서/SLAM 데이터를 웹·언리얼 환경에서 **실시간으로 시각화**한다.
포인트 클라우드 → **복셀화/OctoMap/Occupancy Grid** 변환 파이프라인을 구성하고, 경량 뷰어로 **지도 생성 진행상황(Incremental Mapping)** 을 확인할 수 있게 만든다.
최종적으로 “다이어그램 + 처리과정 문서 + 실행 가능한 데모 페이지(코드/시연)”를 제공한다.



### Flow Chart

<img width="441" height="411" alt="step3 drawio" src="https://github.com/user-attachments/assets/c142734d-facc-47b1-a3cf-0b1366612dc1" />

<img width="2560" height="1440" alt="step3" src="https://github.com/user-attachments/assets/dc0c07c2-bca1-4067-88ee-adc443aa75f7" />

![step3](https://github.com/user-attachments/assets/a52afe7a-57a2-4c84-b023-4081d939cb57)



### Prerequisites


* OS: Ubuntu 22.04
* ROS 2 Humble 
* Gazebo, Octomap


### Dependencies

* Internal (this repo)

  * `agv_pro_ros2/agv_pro_description`
  * `agv_pro_ros2/agv_pro_bringup`
  * `agv_pro_ros2/agv_pro_gazebo`
  * `agv_pro_ros2/agv_pro_base`
  * `agv_pro_ros2/Lslidar_ROS2_driver/lslidar_driver`
  * `agv_pro_ros2/Lslidar_ROS2_driver/lslidar_msgs`
  * [octomap_mapping](https://github.com/OctoMap/octomap_mapping/tree/ros2)

* External

  * `ros-humble-octomap-ros`
  * `ros-humble-octomap-rviz-plugins`



### Build

```bash
# workspace
cd ~/step3
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build (packages of this step)
colcon build --symlink-install \
  --packages-select agv_pro_description agv_pro_bringup agv_pro_gazebo agv_pro_base lslidar_driver lslidar_msgs

# octomap (ros2 branch) built from source 시
colcon build --symlink-install \
  --packages-select octomap_server octomap_mapping \

source install/setup.bash
```



### How to Run

#### Launch Gazebo + Spawn robot + Octomap

```bash
ros2 launch agv_pro_gazebo agv_pro_gazebo.launch.py 
```


### Project Limitations

* **No SLAM / No Loop Closure:** Octomap에 국한됨.
* **성능 제약:** 저사양 GPU/CPU에서 포인트/업데이트 레이트를 낮춰야 안정적.
* **센서 모델 단순화:** 시뮬 LiDAR는 실제 하드웨어보다 잡음/반사 모델이 단순.
