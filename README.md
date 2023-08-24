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

* JaraBot

## 2. 목차
1. 1일차
2. 2일차
3. 3일차

### 1일차
* [개발 환경 설정 확인](./ROS2/환경설정.md)
* [CLI 도구 사용법 Quick 리뷰](./ROS2/turtlesim_ros2_rqt.md)
* [ROS 2 핵심 개념 Quick 리뷰](./ROS2/nodes.md)
* [pub/sub 구현 C++](./ROS2/writingPublisherSubscriber.md)
* [pub/sub 구현 Python](./ROS2/writingPublisherSubscriberPython.md)
* [service 구현 C++](./ROS2/writingServiceClient.md)
* [service 구현 Python](./ROS2/writingServiceClientPython.md)

* [커스텀 msg와 srv 파일 생성하기](./ROS2/CreatingCustomMsgAndSrvFiles.md)
* [custom interface 구현하기](./ROS2/ImplementingCustomInterfaces.md)
* [parameters 사용하기 (C++)](./ROS2/UsingParametersInClassCpp.md)
* [parameters 사용하기 (Python)](./ROS2/UsingParametersInClassPython.md)
* [rosdoctor 사용해서 issues 확인하기](./ROS2/UsingRos2doctorToIdentifyIssues.md)
* [plugins 생성하고 사용하기 (C++)](./ROS2/CreatingAndUsingPluginsCpp.md)
* [action 생성하기]()
* [action server/client 작성 C++](./ROS2/writingActionServerClient.md)
* [action server/client 작성 Python](./ROS2/writingActionServerClient.md)


### 2일차
* [Cartographer 이해](./Cartographer/README.md)
* [Navigation 이해](./Navigation/README.md)
* [시뮬레이션 SLAM](./SLAM/simulation_slam.md)
* [시뮬레이션 Navigation](./SLAM/simulation_nav.md)
* [Jarabot 설정](./jarabot/Setup.md)
* [Jarabot Source Repo](https://github.com/jarabot/jarabot)
* [Jarabot 휠 동작](./jarabot/wheelcontrol.md)
* [Jarabot 키보드로 조작](./jarabot/moving.md)
* [Jarabot SLAM](./SLAM/jarabot_slam.md)
* [Jarabot Navigation](./SLAM/jarabot_nav.md)
* [Jarabot 실습](./Exercise/README.md)

### 3일차
* 대회
    * [달려! 달려! JaraBot](./Competition/GoGoJarabot.md)
    * [잘도 피해가는 JaraBot](./Competition/AvoidanceJarabot.md)
