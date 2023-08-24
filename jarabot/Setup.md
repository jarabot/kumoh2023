# 설정
* RPLiDAR 및 jarabot ROS 2 패캐지 설치
```bash
sudo apt install ros-humble-serial-driver ros-humble-teleop-twist-keyboard

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git    (여기 repo 써야함.)
git clone https://github.com/jebiio/jarabot.git
cd ~/ros2_ws
colcon build

sudo cp ~/ros2_ws/src/jarabot/jarabot_node/rule/99-jarabot.rules /etc/udev/rules.d/

sudo apt install udev

sudo udevadm control --reload-rules
sudo udevadm trigger

sudo reboot  # 리부팅하기

ls /dev/     # /dev/ttyUSB0 확인
```

* Jarabot 관련 node 실행 및 키보드로 움직여보기
```bash
source ~/ros2_ws/install/setup.bash

ros2 launch jarabot_node test.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/keyboard/cmd_vel
```
