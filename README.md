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
* [개발 환경 설정 확인](./ROS2/1_Beginner:CLITools/1_Configuring%20environment.md)
* [CLI 도구 사용법 Quick 리뷰](./ROS2/1_Beginner:CLITools/8_usingRqt_console.md)
* [ROS 2 핵심 개념 Quick 리뷰](./ROS2/1_Beginner:CLITools/3_nodes.md)
* [pub/sub 구현 C++](./ROS2/2_Beginner:ClientLibrary/4\)writingPublisherSubscriber.md)
* [pub/sub 구현 Python](./ROS2/2_Beginner:ClientLibrary/5\)writingPublisherSubscriberPython.md)
* [service 구현 C++](./ROS2/2_Beginner:ClientLibrary/6\)writingActionServerClient.md)
* [service 구현 Python](./ROS2/2_Beginner:ClientLibrary/7\)writingActionServerClientPython.md)
* [커스텀 msg와 srv 파일 생성하기](./ROS2/2_Beginner:ClientLibrary/8\)CreatingCustomMsgAndSrvFiles.md)
* [custom interface 구현하기](./ROS2/2_Beginner:ClientLibrary/9\)implementingCustomInterfaces.md)
* [parameters 사용하기 (C++)](./ROS2/2_Beginner:ClientLibrary/10\)usingParameter\(중복\).md)
* [parameters 사용하기 (Python)](./ROS2/2_Beginner:ClientLibrary/11\)usingParameterPython\(중복\).md)
* [plugins 생성하고 사용하기 (C++)](./ROS2/2_Beginner:ClientLibrary/13\)CreatingAndUsingPluginsCpp.md)
* [action server/client 작성 (C++)](./ROS2/2_Beginner:ClientLibrary/6\)writingActionServerClient.md)
* [action server/client 작성 (Python)](./ROS2/2_Beginner:ClientLibrary/7\)writingActionServerClientPython.md)
---
* [tf2 소개](./ROS2/5_Intermediate:TF2/tf2_IntroducingTF2.md)
* [static broadcaster 작성하기(C++)](./ROS2/5_Intermediate:TF2/tf2_WritingAStaticBroadcaster_C++.md)
* [static broadcaster 작성하기(Python)](./ROS2/5_Intermediate:TF2/tf2_WritingAStaticBroadcaster_Python.md)
* [broadcaster 작성하기(C++)](./ROS2/5_Intermediate:TF2/tf2_WritingABroadcaster_C++.md)
* [broadcaster 작성하기(Python)](./ROS2/5_Intermediate:TF2/tf2_WritingABroadcaster_Python.md)
* [listener 작성하기(C++)](./ROS2/5_Intermediate:TF2/tf2_WritingAListener_C++.md)
* [listener 작성하기(Python)](./ROS2/5_Intermediate:TF2/tf2_WritingAListener_Python.md)
* [프레임 추가하기(C++)](./ROS2/5_Intermediate:TF2/tf2_AddingAFrame_C++.md)
* [프레임 추가하기(Python)](./ROS2/5_Intermediate:TF2/tf2_AddingAFrame_Python.md)
* ['time'기능 사용하기(C++)](./ROS2/5_Intermediate:TF2/tf2_UsingTime_C++.md)
* ['time'기능 사용하기(Python)](./ROS2/5_Intermediate:TF2/tf2_UsingTime_Python.md)
* [시간여행 하기(C++)](./ROS2/5_Intermediate:TF2/tf2_TravelingInTime_C++.md)
* [시간여행 하기(Python)](./ROS2/5_Intermediate:TF2/tf2_TravelingInTime_Python.md)
* [디버깅](./ROS2/5_Intermediate:TF2/tf2_Debugging.md)
* [쿼터니언 기초](./ROS2/5_Intermediate:TF2/tf2_QuaternionFundamentals.md)
* [메세지 필터를 이용한 'stamped' 자료형 사용하기](./ROS2/5_Intermediate:TF2/tf2_UsingStampedDatatypesWith_tf2_ros_MessageFilter.md)

### 2일차
* [Cartographer 이해](./Cartographer/README.md)
* [Navigation 이해](./Navigation/README.md)
* [시뮬레이션 SLAM](./SLAM/simulation_slam.md)
* [시뮬레이션 Navigation](./SLAM/simulation_nav.md)
* [Jarabot 설정](./jarabot/Setup.md)
* [PC 설정](./jarabot/PCSetup.md)
* [설정완료 확인](./jarabot/Done.md) 
* [Jarabot Source Repo](https://github.com/jarabot/jarabot)
* [Jarabot 휠 동작](./jarabot/wheelcontrol.md)
* [Jarabot 키보드로 조작](./Exercise/MovingJarabot.md)
* [JaraBot 지도 생성하기](./Exercise/BuildMap.md)
* [Jarabot Navigation 하기](./Exercise/DoNavigation.md)

### 3일차
* 대회
    * [달려! 달려! JaraBot](./Competition/GoGoJarabot.md)
    * [잘도 피해가는 JaraBot](./Competition/AvoidanceJarabot.md)
