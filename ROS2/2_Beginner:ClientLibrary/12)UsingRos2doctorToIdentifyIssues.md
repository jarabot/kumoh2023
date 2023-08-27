# [ros2doctor 사용하여 설정 issues 확인하기](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Getting-Started-With-Ros2doctor.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. ROS2 설치 설정 검사하기
   2. 실행중인 system 검사하기
   3. 전체 리포트 가져오기
5. 요약

## 목표
* ros2doctor 도구를 사용하여 ROS2 설정에 문제가 있는지를 확인 
## 배경지식
* ROS2 설치된 환경설정으로 실행시 예상대로 동작하지 않는 경우에 rosdoctor 도구를 이용하여 설정을 검사할 수 있다.
* ros2doctor은 ROS2의 모든 것들을 검사한다
  * platform
  * version
  * network
  * environment
  * 실행 system
  * ...

* 만약 검사시에 문제가 있는 경우 다음을 출력
  * 발생 가능한 errors
  * 문제의 원인

## 사전준비
* ros2doctor는 ros2cli package의 일부이다.
* 따라서 ros2cli가 설치되면 ros2doctor를 사용할 수 있다.
* 이 튜터리얼은 turtlesim을 사용하여 보여준다.  

## 실습
###  1. ROS2 설치 설정 검사하기
* ros2doctor로 ROS2 설정을 검사해보자.
* 새 터미널을 열고 ROS2 환경을 source 하자. 
* 아래 명령을 실행한다.
```
ros2 doctor
```
* 하는 일
  * 모든 설정 모듈을 검사
  * 문제가 있으면 warning 및 errors 반환
* ROS2 설정에 아무런 문제가 없으면 아래와 같은 메시지가 나온다.
```
All <n> checks passed
```
* 하지만 warning 몇 개 나오는 경우에는 그렇게 문제가 되지는 않는다.
* UserWarning이라고해서 반드시 설정에 문제가 있다는 의미는 아니다.
  * 설정이 조금 다르게
* warning은 다음과 같은 형태다
```
<path>: <line>: UserWarning: <message>
```
* 예제) 안정된 ROS2 배포판이 아닌 경우 ros2doctor는 다음과 같은 warning이 발생
```
UserWarning: Distribution <distro> is not fully supported or tested. To get more consistent features, download a stable version at https://index.ros.org/doc/ros2/Installation/
```
* ros2doctor로 warning만 있는 경우라면 All <n> checks passed 메시지가 나온다.

* UserWarning: ERROR로 표시되면 fail로 간주된다.
```
1/3 checks failed

Failed modules:  network
```
* error가 의미하는 것은 ROS2에 대한 기능이나 설정이 빠진 부분이 있다는 의미이다.

###  2. 실행중인 system 검사하기
* 실행 중인 ROS2 시스템 검사하기
* 실행 중인 시스템에 대해서 ros2doctr 동작을 보기 위해서 TurtleSim을 실행해보자. 

* 새 터미널을 열고 아래 명령을 실행한다.
```
ros2 run turtlesim turtlesim_node
```
* 새 터미널을 열고 아래 명령을 실행한다.
```
ros2 run turtlesim turtle_teleop_key
```
* 이제 ros2doctor 를 다시 터미널에서 실행하자. 
* 이전에 봤던 warning이 보일 것이다. 
* 아래는 새로운 warning을 보여준다.
```
UserWarning: Publisher without subscriber detected on /turtle1/color_sensor.
UserWarning: Publisher without subscriber detected on /turtle1/pose.
```
* 의미하는 바는 /turtlesim node가 2개 data를 publish하는데 subscribe하는 node가 없다는 것이고 ros2doctor가 판단하기에 issues가 될 수 있다고 판단한 것이다.

* 명령으로 /color_sensor와 /pose topics을 echo 명령을 실행하면, 이 warning은 사라진다. 왜냐하면 subscriber가 있기 때문이다.

* 새 터미널을 열어서 echo 명령을 실행해 보자.
```
ros2 topic echo /turtle1/color_sensor
```

```
ros2 topic echo /turtle1/pose
```

* 다음으로 ros2doctor를 다시 실행해보자.
* publisher without subscriber warning은 사라졌다.

* 이제 turtlesim window나 teleop 중에 하나를 종료시키고 ros2doctor를 실행해 보자. 다른 topics에 대해서 publisher without subscriber 혹은 subscriber without publisher 에 대한 warning이 나타난다.

* 많은 nodes를 가지고 있는 복잡한 시스템내에서 ros2doctor는 유용하지 않을 수 있다.
  * 왜???
###  3. 전체 리포트 가져오기
* ros2doctor을 실행할때 인자로 --report를 사용하면 상세하게 warning 내용을 알려준다.
* --report를 사용하는 경우
  * network 설정 관련 이슈
  * 정확히 어느 부분에서 설정이 잘못되어 warning이 나는지를 알고 싶을때
* ROS2 커뮤니티에 도움을 요청할때 환경에 대한 정보를 같이 제공해주면 도움 얻기가 편하다.

* full report를 얻으려면 다음 명령을 수행한다.
```
ros2 doctor --report
```

* 5개 그룹으로 분류된 정보 목록을 출력한다.
```
NETWORK CONFIGURATION
...

PLATFORM INFORMATION
...

RMW MIDDLEWARE
...

ROS 2 INFORMATION
...

TOPIC LIST
...
```

* ros2 doctor를 실행해서 얻은 warning과 크로스체크가 가능하다.
* 예제로 만약 ros2doctor가 "not fully supported or tested"라는 warning을 출력하면 ROS2 INFORMATION 섹션을 살펴볼 수 있겠다. 
```
distribution name      : <distro>
distribution type      : ros2
distribution status    : prerelease
release platforms      : {'<platform>': ['<version>']}
```
* 여기서는 distribution status가 prerelease이기 때문에 fully supported가 아닌 이유라고 판단할 수 있는 근거가 된다.

## 요약
* ros2doctor는 ROS2 설정에 문제가 있는지 여부를 알려준다.
* --report 인자를 사용하여 실행하면 warning의 의미에 대해서 좀더 자세히 알려준다.
* ros2doctor는 디버깅 도구는 아니다. 
  * 즉 코드나 구현에 문제가 있는지를 알려주는 것이 아니다.
* [ros2doctor 문서 링크](https://github.com/ros2/ros2cli/tree/humble/ros2doctor)