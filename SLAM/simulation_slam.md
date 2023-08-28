# Turtlebot4 Simulation 환경 설정
* 목차
  1. Dev Tools
  2. Iginition Gazebo
  3. Install from Source or Package

## Dev Tools
```bash
sudo apt install ros-dev-tools
```

## Iginition Gazebo
```bash
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```

## Install from Source or Package
```bash
mkdir -p ~/turtlebot4_ws/src 
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b humble

cd ~/turtlebot4_ws
rosdep install --from-path src -yi

colcon build --symlink-install
```

## Gazebo Ignition 실행
* 기본 설정으로 실행
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

* Nav2와 동기화 SLAM 실행
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```

[![Video](http://img.youtube.com/vi/go7jszqFSi0/0.jpg)](http://www.youtube.com/watch?v=go7jszqFSi0)

* 사용 방법
  * 실행된 Gazebo에서 왼쪽 아래 'Play' 아이콘 클릭 
  * HMI 조작
    * '3', '4' : 위/아래 메뉴 조작
    * '1', '2' : '1'은 선택, '2'는 취소


## Gazebo Ignition GUI toolbox
* turtlebot4_ignition_toolbox package
  * HMI node 역할
    * turtlebot4_node와 ros_ign_birdge 사이의 bridge 역할
      * Turtlebot4 messages를 표준 message로 변환


