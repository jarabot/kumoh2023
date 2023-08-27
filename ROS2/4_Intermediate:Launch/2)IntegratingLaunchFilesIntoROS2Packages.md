# [launch 파일을 ROS2 packages로 통합시키기](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
5. 요약

## 목표
* launch 파일을 ROS2 package에 추가하기

## 배경지식
* 이전 튜터리얼에서 lauch 파일 작성하는 방법을 배웠다.
* launch 파일을 기존 package에 추가하는 방법을 배워보자.
* lauch 파일의 일반적인 패턴을 익혀보자.

## 사전준비
* ROS2 package 생성하는 방법
* 새 터미널 열때마다 source 명령

## 실습
### 1. package 생성하기
* package에 대한 workspace 생성하기
```bash
mkdir -p launch_ws/src
cd launch_ws/src
```
* C++ package 생성
```
ros2 pkg create cpp_launch_example --build-type ament_cmake
```

* python package 생성
```
ros2 pkg create py_launch_example --build-type ament_python
```

### 2. launch 파일을 가지는 구조 생성
* package의 모든 launch 파일은 launch 디렉토리에 저장
* packge의 최상위에서 launch 디렉토리 생성
* C++ package인 경우
```m
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```
* Python package인 경우
```
src/
  py_launch_example/
    package.xml
    py_launch_example/
    resource/
    setup.py
    setup.cfg
    test/
```
* colcon이 launch 파일을 찾을 수 있도록 하기 위해서 Python의 setup tools에게 setup의 data_files 파라미터를 사용해서 lauch 파일 알려준다.
```python
# setup.py
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

### launch 파일 작성하기
* Python launch 파일
```python
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
```

* XML launch 파일
```xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="talker"/>
</launch>
```

### 4. launch 파일을 빌드 및 실행하기
* workspace의 상위 레벨로 가서 빌드하기
```bash
colcon build
```
* colcon build가 성공적으로 끝나면 workspace에 대해서 source한다.
* 이제 launch 파일을 다음과 같이 실행할 수 있다.
* Python package 실행
```bash
ros2 launch py_launch_example my_script_launch.py
```
* XML launch 실행
```bash
ros2 launch py_launch_example my_script_launch.xml
```
## 문서
* [launch 문서](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)
