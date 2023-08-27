# [substitutions 사용하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html)
1. 목표
2. 배경지식
3. 사전준비
4. substitutions 사용하기
   1. package 생성 및 설정하기
   2. Parent launch 파일
   3. Substitutions 예제 launch 파일
   4. package 빌드하기
5. Launching 예제
6. launch arguments 수정하기
7. 문서
8. 요약

## 목표
* 
## 배경지식
* 
## 사전준비
* 
## substitutions 사용하기
###   1. package 생성 및 설정하기
* launch_tutorial이라는 ament_python build_type의 새로운 package를 생성한다.
```
ros2 pkg create launch_tutorial --build-type ament_python
```
* package의 내부는 launch라는 디렉토리를 생성한다.
```
mkdir launch_tutorial/launch
```
* 마지막으로 launch 파일이 설치되도록 package의 setup.py에 추가한다.
```python
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```
###   2. Parent launch 파일
* argument를 다른 launch 파일에 전달하는 launch 파일을 생성해보자.
* * launch_tutorial package의 launch 디렉토리 내부에 example_main.launch.py 파일을 생성하자.
```python
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```
* example_main.launch.py 파일 내에서 FindPackageShare substitution는 launch_tutorial package에 대한 path를 찾는데 사용한다.
* PathJoinSubstitution substitution는 path를 example_substitutions.launch.py 파일 이름을 가지는 package path로 join한다.
```python
PathJoinSubstitution([
    FindPackageShare('launch_tutorial'),
    'example_substitutions.launch.py'
])
```
* turtlesim_ns와 use_provided_red arguments를 가지는 launch_arguments dictionary는 IncludeLaunchDescription action에 전달된다.  
* TextSubstitution substitution은 colors dictionary내에서 background_r key 값으로 new_background_r argument를 정의한다.
```python
launch_arguments={
    'turtlesim_ns': 'turtlesim2',
    'use_provided_red': 'True',
    'new_background_r': TextSubstitution(text=str(colors['background_r']))
}.items()
```

###   3. Substitutions 예제 launch 파일
* 이제 동일 디렉토리 내부에 example_substitutions.launch.py 파일을 생성하자.
```python
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```
###   4. package 빌드하기
* workspace로 가서 package를 빌드하자.
```
colcon build
```
* 빌드된 후에 workspace를 source 하자.

## Launching 예제
* 이제 ros2 launch 명령을 이용하여 example_main.launch.py 파일을 lauch할 수 있다.
```
ros2 launch launch_tutorial example_main.launch.py
```
* 위 명령은 다음과 같을 작업을 수행한다.
   1. blue background로 turtlesim node를 구동시키자.
   2. 2번째 turtle을 spawn한다.
   3. 색상을 purple로 변경한다.
   4. 제공된 background_r 인자가 200이고 user_provided_red 인자가 True인 경우에 2초 후에 pink로 색상이 변경된다.

## 6. launch arguments 수정하기
* 제공되는 launch arguments를 변경하고자 한다면 example_main.launch.py 내부에 launch_arguments dictionary 내에세ㅓ 인자들을 업데이트하거나 원하는 arguments를 가진 example_substitutions.launch.py를 launch할 수 있다.
* launch 파일에 주어진 arguments를 볼려면 아래 명령을 실행한다.
```
ros2 launch launch_tutorial example_substitutions.launch.py --show-args
```
* launch 파일과 주어진 기본값의 arguments를 보여준다.
```
Arguments (pass arguments as '<name>:=<value>'):

    'turtlesim_ns':
        no description given
        (default: 'turtlesim1')

    'use_provided_red':
        no description given
        (default: 'False')

    'new_background_r':
        no description given
        (default: '200')
```
* 이제 다음과 같이 원하는 arguments를 launch 파일에 전달할 수 있다.
```
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

## 문서
* [launch 관련 문서](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)
## 요약
* 이 튜터리얼에서 launch 파일 내에서 substitutions을 사용하는 방법에 대해서 배웠다. 
* 재사용 가능한 launch 파일을 생성하기 위해서 가능성과 기능에 대해서 배웠다.
* 이제 'launch 파일내에서 event handlers 사용하기'에 대해서 좀더 배워보자. 이를 이용하면 동적으로 launch 파일을 수정하는데 사용할 수 있는 복잡한 rule을 적용할 수 있다.
