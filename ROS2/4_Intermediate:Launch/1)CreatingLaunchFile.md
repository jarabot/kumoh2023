# [launch 파일 생성하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. setup
   2. launch 파일 작성하기
   3. ros2 launch
   4. rqt_graph로 시스템 
5. 요약

## 목표
* launch 파일 만들어 보기 
* launch 파일로 ROS2 실행시켜보기
## 배경지식
* ROS2의 launch가 하는 일
  * ROS2 system 실행
    * 실행자 지정
    * 환경 설정(configuration)
    * 실행 프로그램에게 인자(arguments)로 전달
    * 실행시킨 processes의 상태 모니터링
    * 이 processes의 state 변경에 대한 반응 및 리포팅
* Launch 파일 포맷
  * Python
  * XML
  * YAML
* launch_ros package 사용
  * launch 명령에 대한 인터페이스 제공(code로 launch 명령 사용하는 것과 동일 효과)
  * launch_ros는 실제로 내부에서는 launch 명령을 사용하는 것과 같다.

## 사전준비
* rqt_graph와 turtlesim package를 사용
* 새 터미널 열면 ROS2 source 해주기

## 실습
### 1. setup
* launch 파일을 저장할 새로운 디렉토리 생성하기
```
mkdir launch
```

### 2. launch 파일 작성
* turtlesim package와 실행자를 이용하는 ROS2 launch 파일 예시 
* 3가지 방식
  * Python
  * XML
  * YAML
* Python (launch/turtlesim_mimic_launch.py에 생성)
```python
# launch/turtlesim_mimic_launch.py 파일
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```
* XML (launch/turtlesim_mimic_launch.xml 파일)
```xml
<launch>
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2"/>
  <node pkg="turtlesim" exec="mimic" name="mimic">
    <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
    <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
  </node>
</launch>
```
### 2.1 launch 파일 살펴보기
* 몇개의 nodes를 실행하는가?
* 어느 package의 nodes인가?

### 3. ros2 launch
* 위에서 생성한 launch 파일을 실행하기 위해서
  * 생성한 디렉토리(launch)로 진입
  * ros2 launch 명령으로 실행
* Python 실행
```
cd launch
ros2 launch turtlesim_mimic_launch.py
```
* XML 실행
```xml
cd launch
ros2 launch turtlesim_mimic_launch.xml
```
* Note
  * launch 파일을 바로 실행하는 것도 가능
* launch 사용 문법
```bash
ros2 launch <package_name> <launch_file_name>
```
* package 내부에 package.xml에서 ros2launch package에 의존성 추가
```xml
<exec_depend>ros2launch</exec_depend>
```
* 빌드된 package에 대해서 ros2 launch 명령이 유효하다는 것을 설정 

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [11714]
[INFO] [turtlesim_node-2]: process started with pid [11715]
[INFO] [mimic-3]: process started with pid [11716]
```

* action 내에서 시스템을 보기 위해서, 새 터미널을 열고 아래와 같이 /turtlesim1/turtle1/cmd_vel topic 에 대해서 ros2 topic pub 명령을 실행하자. 첫번째 turtle이 움직이게 된다.
```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

* 양쪽 turtles가 동일한 path를 따라 움직이는 것을 볼 수 있다.
![](https://docs.ros.org/en/humble/_images/mimic.png)

### 4. rqt_graph로 시스템 내부 살펴보기
* 시스템이 실행되는 동안 새로운 터미널 열기
* rqt_graph 명령 실행
  * launch 파일 내에 있는 nodes 사이의 관계를 이해하는 도구로 사용
```
rqt_graph
```

![](https://docs.ros.org/en/humble/_images/mimic_graph.png)

* 감춰진 node(ros2 topic pub으로 실행한 터미널)가 /turtlesim1/turtle1/cmd_vel topic으로 data를 publish하고 /turtlesim1/sim node가 이를 수신한다.
* graph의 나머지는 부분은 이전에 봤던 것과 같이 mimic이 /turtlesim1/sim의 pose topic을 수신하고 /turtlesim2/sim의 velocity command topic에게 publish한다.

## 요약
* Launch 파일
  * 여러 nodes들을 한꺼번에 실행
  * 실행에 필요한 configuration 가능
* 파일 형식
  * Python
  * XML
  * YAML
* 명령
  * ros2 launch 명령