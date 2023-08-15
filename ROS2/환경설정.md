# 환경설정
1. 개요
2. 실습

## 1. 개요
* workspace
  * 우리가 ROS 2를 개발하고 있는 위치
* uderlay
  * /opt/ros/humble/
* overlay
  * ~/ros2_ws/
* 우리가 입력한 명령어가 실행되게 하기!

## 2. 실습
### 2-1 setup 파일을 source 하기
* 새로운 터미널을 열어서 아래 실행
```bash
source /opt/ros/humble/setup.bash
```

### 2-2 터미널 시작시 자동으로 setup.bash 파일 source 시키기
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2-3 환경변수 확인하기
```bash
printenv | grep -i ROS
```

* 결과
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

### 2-3-1 ROS_LOCALHOST_ONLY 변수
* ROS_LOCALHOST_ONLY 변수 설정하기
```bash
export ROS_LOCALHOST_ONLY=1
```

* 터미널 시작시 자동으로 설정되도록 하기
```bash
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```