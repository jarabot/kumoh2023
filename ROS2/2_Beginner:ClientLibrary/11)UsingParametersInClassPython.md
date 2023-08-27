# [parameters 사용하기 Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. package 생성하기
   2. C++ node 작성하기
   3. 빌드 및 실행
5. 요약

## 목표
* ROS parameters를 가지는 class를 생성하고 실행하기 (Python)
## 배경지식
* node를 생성하고 나면 launch 파일에서 설정이 가능한 parameters를 추가해야하는 경우가 있다.
* 이 튜터리얼에서 Python class내부에서 이런 parameters를 생성하는 방법을 배워보자.
* launch 파일내에서 이 parameters를 설정하는 방법에 대해서 배워보자.

## 사전준비
* 이전 튜터리얼에서 배운 workspace, package 생성 방법 이해
* ROS2에서 parameters와 기능 이해

## 실습
###  1. package 생성하기
* 터미널 열고 ~/ros2_ws/src 폴더로 이동후 아래 명령 실행
```bash
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy
```

#### 1.1 package.xml 업데이트
* --dependencies 옵션으로 package를 생성하였으므로 package.xml 파일이나 CMakeLists.txt 파일에 자동으로 의존성이 추가된다.

* package.xml 에 정보 추가(email, license, description)
```xml
<description>Python parameter tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

### 2. Python node 작성하기
* ~/ros2_ws/src/python_parameters/src 디렉토리 내부에 python_parameters_node.py 파일 생성하고 아래 내용을 복사해서 붙여넣자.
```python
import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')

        self.declare_parameter('my_parameter', 'world')

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

* Parameter 생성 : self.declare_parameter('my_parameter', 'world')
* 1초에 1번 실행되는 timer_callback 함수 

#### 2.1.1 ParameterDescriptor 추가하기
```python
# ...

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')

        # parameterdescriptor 추가
        from rcl_interfaces.msg import ParameterDescriptor
        my_parameter_descriptor = ParameterDescriptor(description='This parameter is mine!')

        // declare_parameter 함수에 추가
        self.declare_parameter('my_parameter', 'world', my_parameter_descriptor)

        self.timer = self.create_timer(1, self.timer_callback)
```

* prameter type과 description을 확인하는 명령
```bash
ros2 param describe /minimal_param_node my_parameter
```

#### 2.2 entry point 추가하기
* setup.py 파일 열고 아래 내용 추가하기
* package.xml 파일과 같이 maintainer, maintainer_email, description, license 항목이 일치 시킨다.
```python
maintainer='YourName',
maintainer_email='you@email.com',
description='Python parameter tutorial',
license='Apache License 2.0',
```

* entry_points 필드의 console_scripts 영역 내부에 아래 코드를 추가한다.
```python
entry_points={
    'console_scripts': [
        'minimal_param_node = python_parameters.python_parameters_node:main',
    ],
},
```

### 3. 빌드 및 실행
* ~/ros2_ws 폴더로 이동 후 아래 명령 실행(의존성 검사)
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

* 새 package 빌드하기
```bash
cd ~/ros2_ws
colcon build --packages-select python_parameters
```

* setup.bash 실행
```bash
source install/setup.bash
```

* node 실행하기
```bash
ros2 run python_parameters minimal_param_node
```

* 결과
```
[INFO] [parameter_node]: Hello world!
```

#### 3.1 console 이용하여 변경하기
* node를 실행시키자
```bash
ros2 run python_parameters minimal_param_node
```

* 새로운 터미널 열고 setup.bash 실행 후 아래 명령 실행
```bash
ros2 param list
```

* 결과
```
/my_parameter
```

* console에서 parameter 변경하기
```bash
ros2 param set /minimal_param_node my_parameter earth
```

* 결과
```
Set parameter successful
[INFO] [minimal_param_node]: Hello earth!
```

#### 3.2 launch 파일 이용하여 변경하기
* launch 파일 내부에서 parameter 설정이 가능.
* 먼저 launch 디렉토리를 추가
* ~/ros2_ws/src/python_parameters/ 디렉토리 내부에 launch 디렉토리를 생성한다.
* 이 디렉토리 내부에 python_parameters_launch.py 파일을 생성한다.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='minimal_param_node',
            name='custom_minimal_param_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])
```
* setup.py 파일에 launch 파일을 추가하기 위해서 data_files parameter를 추가 
```python
import os
from glob import glob
# ...

setup(
  # ...
  data_files=[
      # ...
      (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ]
  )
```

* 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select python_parameters
```

* 새 터미널에서 setup 파일을 source하기
```bash
cd ~/ros2_ws
source install/setup.bash
```

* launch 파일을 이용해서 node 실행
```bash
ros2 launch python_parameters python_parameters_launch.py
```

## 요약
* custom parameter를 가지는 node를 생성하고 실행
