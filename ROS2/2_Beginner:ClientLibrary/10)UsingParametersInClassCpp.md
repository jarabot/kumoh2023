# [parameters 사용하기 C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. package 생성하기
   2. C++ node 작성하기
   3. 빌드 및 실행
5. 요약

## 목표
* ROS parameters를 가지는 class를 생성하고 실행하기 (C++)
* 이 튜터리얼에서 C++ class로 parameters를 생성하는 방법
* launch 파일에서 parameters를 설정하는 방법에 대해서 배워보자.
## 배경지식
* 이전 튜터리얼에서 배운 workspace 생성 방법과 package 생성하는 방법
* parameters 개념
## 사전준비
*  workspace, package 생성 방법 이해
* ROS2에서 parameters와 기능 이해
## 실습
###  1. package 생성하기
* 새 터미널 열기
* ROS2 환경 source 명령 실행하기
* ros2 명령 동작 여부 확인하기
* 새 workspace 생성하기 - [링크](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#new-directory)
* packages를 src 디렉토리 내부에 생성(ros2_ws/src 에서 생성)을 위해 아래 명령 실행
```
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```
* 실행 결과로 cpp_parameters package를 생성, 필요한 파일 및 디렉토리 생성되었다는 메시지가 출력된다.
* package.xml과 CMakeLists.txt 파일에 자동으로 의존성 파일이 추가된다. 의존성 추가를 위해서 --dependencies 인자를 이용하여 명령을 수행하였다.

### 1.2 package.xml 업데이트
* package 생성할때 --dependencies를 사용하였기 때문에 자동 생성되었다. 따라서 수동으로 package.xml과 CMakeLists.txt에 의존성을 추가하지 않아도 되었다.
* package.xml 파일에 아래와 같이 설명, 작성자 메일, 이름, 라이센스 정보를 입력하자.
```xml
<description>C++ parameter tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

###  2. C++ node 작성하기
* ros2_ws/src/cpp_parameters/src 디렉토리 내부에 cpp_parameters_node.cpp 파일 생성하고 아래 내용을 복사해서 붙여넣자.
```c++
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    this->declare_parameter("my_parameter", "world");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    std::string my_param =
      this->get_parameter("my_parameter").get_parameter_value().get<std::string>();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
```
### 2.1 code 들여다보기
* #include 구문
  * 의존하는 package
* 생성자
  * parameter 이름 : my_parameter
  * value : world
  * parameter type : value로부터 추론 (string type)
  * timer : 1초 주기로 호출. timer_callback 함수가 호출된다.
* timer_callback 함수
  * get_parameter() : node로부터 my_parameter parameter 가져오기
  * RCLCPP_INFO 함수 : message 로깅
  * set_parameters 함수 : node의 parameter 설정
* main()
  * ROS2 초기화로 MinimalParam class 인스턴스 생성
  * rclcpp::spin 이 실행되며 node에서 data 처리가 시작된다.

### 2.2 실행자 추가하기
* CMakeLists.txt 파일 열기
* find_package(rclcpp REQUIRED) 아래에 다음 코드를 추가
```cmake
add_executable(minimal_param_node src/cpp_parameters_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp)

install(TARGETS
  minimal_param_node
  DESTINATION lib/${PROJECT_NAME}
)
```

###  3. 빌드 및 실행
* workspace(ros2_ws) 아래에서 rosdep 실행한다.
  * 빌드하는데 필요한 의존성이 모두 있는지 검사
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

* ros2_ws로 가서 새 package를 빌드하기
```
colcon build --packages-select cpp_parameters
```

* 새 터미널 열기. ros2_ws로 이동해서 setup 파일을 source 하기
```
. install/setup.bash
```
* node 실행하기
```
ros2 run cpp_parameters minimal_param_node
```
* 터미널은 매 초마다 다음 메시지를 출력해야한다.
```
[INFO] [minimal_param_node]: Hello world!
```

* 이제 parameter의 기본값을 볼수 있지만 직접 우리의 값으로 설정할 수도 있다. 설정하는 2가지 방법에 있다.

### 3.1 console 이용하여 변경하기
* [parameter 튜터리얼](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)에서 배운 지식을
* 우리가 위에서 만든 node에 적용해보자.
* 적용을 위해서는 해당 node가 실행 중이여야 한다.
```
ros2 run cpp_parameters minimal_param_node
```

* 새 터미널을 열고 ros2_ws에 있는 setup 파일을 source한다. 그리고 난 후에 아래 명령을 실행한다.
```
ros2 param list
```

* 커스텀 파라미터인 my_parameter를 볼 수 있다. 이를 변경하기 위해서 아래 명령을 수행한다.
```
ros2 param set /minimal_param_node my_parameter earth
```

* 'Set parameter successful'이라는 출력이 나오면 잘 실행이 된 것이다.
* 다른 터미널에서는 '[INFO] [minimal_param_node]: Hello earth!'라는 메시지로 출력이 바뀐것을 볼 수 있다.

### 3.2 launch 파일 이용하여 변경하기
* launch 파일 내에 parameter 설정이 있다. 하지만 먼저 launch 디렉토리를 추가한다.
* ros2_ws/src/cpp_parameters/ 디렉토리 내부에 launch 디렉토리를 생성한다. 
* 이 디렉토리 내부에 cpp_parameters_launch.py 파일을 생성한다. 
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
```

* 여기서 my_parameter가 earth로 설정되어 있는 것을 볼 수 있다. (minimal_param_node node를 launch 시킬때)
* 아래와 같이 2줄을 추가하면 console에 출력된다.
```
output="screen",
emulate_tty=True,
```

* 이제 CMakeLists.txt 파일을 열어보자. 이전에 추가했던 라인의 아래 라인에 추가하자.
```
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

* console을 열고 ros2_ws 디렉토리로 이동한 후에 아래 명령으로 새로운 package를 빌드한다.
```
colcon build --packages-select cpp_parameters
```

* 새 터미널에서 setup 파일을 source 한다.
```
. install/setup.bash
```

* 이제 launch 파일을 이용해서 좀전에 빌드한 node를 실행해보자.
```
ros2 launch cpp_parameters cpp_parameters_launch.py
```

* 이 터미널은 아래와 같은 메시지가 출력된다.
```
[INFO] [custom_minimal_param_node]: Hello earth!
```

## 요약
* 커스텀 parameter를 가지는 node를 생성해보았다.
* launch 파일과 console로 설정할 수 있다.
* 빌드 및 실행을 위해서 의존성, 실행자, launch 파일을 package 설정 파일에 추가했다.
  