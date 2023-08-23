# JaraBot을 이용한 ROS 2 SLAM 이해 - (금오공대 2023)

![](https://avatars.githubusercontent.com/u/142007781?s=400&u=3051e39dae4470600bf72bf50abb36fc15692e12&v=4)

1. 준비물
2. 목차
## 1. 준비물
* Ubuntu 22.04 설치된 노트북
  * 원활한 강의 진행을 위해서 반드시 Ubuntu 22.04를 설치
  * 가상머신이나 VMWare를 통한 Ubuntu 22.04 설치 금지!!

* ROS 2 humble 버전 설치
  * 설치방법 : [링크 참조](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## 2. 목차
1. 1일차
2. 2일차
3. 3일차

### 2-1 1일차
* [개발 환경 설정 확인](./ROS2/환경설정.md)
* [CLI 도구 사용법 Quick 리뷰](./ROS2/turtlesim_ros2_rqt.md)
* [ROS 2 핵심 개념 Quick 리뷰](./ROS2/nodes.md)
* [ROS 2 pub/sub 구현 C++/Python](./ROS2/writingPublisherSubscriber.md)
* [ROS 2 service 구현 C++/Python](./ROS2/writingServiceClient.md)
* [ROS 2 action 구현 C++/Python](./ROS2/writingActionServerClient.md)
* 기타 등등 추가

### 2-1 2일차
* [Cartographer 이해](./Cartographer/README.md)
* [Navigation 이해](./Navigation/README.md)
* [시뮬레이션 SLAM](./SLAM/simulation_slam.md)
* [시뮬레이션 Navigation](./SLAM/simulation_nav.md)
* [Jarabot 설정](./jarabot/setup.md)
* [Jarabot Source Repo](https://github.com/jarabot/jarabot)
* [Jarabot 휠 동작](./jarabot/wheelcontrol.md)
* [Jarabot 키보드로 조작](./jarabot/moving.md)
* [Jarabot SLAM](./SLAM/jarabot_slam.md)
* [Jarabot Navigation](./SLAM/jarabot_nav.md)

### 2-1 3일차
* 대회
    * [달려! 달려! JaraBot](./Competition/GoGoJarabot.md)
    * [잘도 피해가는 JaraBot](./Competition/AvoidanceJarabot.md)
