# [parameters 변경 모니터링](https://docs.ros.org/en/humble/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. package 생성하기
   2. c++ node 작성하기
   3. 빌드 및 실행
5. 요약

## 목표
* parameter가 변경을 모니터링하는 기능 구현에 사용하는 ParameterHandler class 사용방법을 익혀보자.

## 배경지식
* parameters 변경시 대응
  * 자신 node의 parameter 변경시 대응
  * 다른 nodes의 parameter 변경시 대응
* ParameterEventHandler class로 쉽게 parameter 변경을 모니터링(listen)하고 변경시 대응 코드 작성
* C++ 버전의 ParameterEventHandler class 사용법을 배워보자.
  * 다른 nodes parameters 변경에 대한 모니터링
  * 자신 node에서의 parameter 변경에 대한 모니터링

## 사전준비
* 사전학습
  * parameters 이해하기
  * C++ class에서 parameters 사용하기
* ROS2 Galactic 배포판을 실행해야만 한다.***

## 실습
* 이 튜터리얼에서 샘플 코드를 포함하는 새로운 package를 생성한다.
* ParameterEventHandler class를 사용하기 위해서 C++로 작성한다.
* 결과 코드를 테스트한다.

### 1. package 생성하기
* 새로운 터미널 열기
* ROS2 환경 source 하기
* ros2 명령 정상 동작 확인하기
* ros2_ws라는 새로운 workspace를 생성한다. [생성 명령](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#new-directory)
* package는 src 디렉토리 아래에 생성되어야 한다. (workspace 아래에 생성하지 않도록 주의!!)
* ros2_ws/src로 이동해서 다음과 같이 package를 생성한다.
```
ros2 pkg create --build-type ament_cmake cpp_parameter_event_handler --dependencies rclcpp
```
* 명령의 결과로 cpp_parameter_event_handler package가 생성되었고 필요한 파일과 디렉토리가 생성되었다는 메시지가 표시
* --dependencies 인자는 자동으로 필요한 의존성을 추가한다. 추가되는 파일 package.xml과 CMakeLists.txt 2개 파일에 자동 추가됨

### 1.1 Update
* package 생성할때 --dependencies 옵션을 사용하였으므로 수동으로 package.xml과 CMakeLists.txt 에 넣을 필요가 없다. 다음과 같은 정보를 package.xml에 추가하자.
```xml
<description>C++ parameter events client tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

### 2. C++ node 작성하기
* ros2_ws/src/cpp_parameter_event_handler/src 디렉토리 내부에 parameter_event_handler.cpp라는 새로운 파일을 생성하고 아래 내용을 붙여넣자.
```c++
#include <memory>

#include "rclcpp/rclcpp.hpp"

class SampleNodeWithParameters : public rclcpp::Node
{
public:
  SampleNodeWithParameters()
  : Node("node_with_parameters")
  {
    this->declare_parameter("an_int_param", 0);

    // Create a parameter subscriber that can be used to monitor parameter changes
    // (for this node's parameters as well as other nodes' parameters)
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Set a callback for this node's integer parameter, "an_int_param"
    auto cb = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
          this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
          p.get_name().c_str(),
          p.get_type_name().c_str(),
          p.as_int());
      };
    cb_handle_ = param_subscriber_->add_parameter_callback("an_int_param", cb);
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
  rclcpp::shutdown();

  return 0;
}
```
 
### 2.1 코드 살펴보기
* 첫번째 줄인 #include <memory>는 std:make_shared template를 사용하기 위해서 추가하였다. 
* 다음 #include "rclcpp/rclcpp.hpp"는 rclcpp 인터페이스가 제공하는 기능을 참조하기 위해서 추가하였다. 여기에는 ParameterEventHandler class도 추가된다.
* class 선언 후에 코드는 SampleNodeWithParameters class를 정의한다. class의 생성자에 an_int_param이라는 정수 파라미터를 선언하였고 기본값은 0이다. 
* 다음으로 ParameterEventHandler를 생성하여 parameters의 변경을 모니터링하는데 사용한다.
* 마지막으로 lambda 함수를 생성하고 an_int_param이 업데이트 될때마다 호출되도록 callback으로 설정한다.

### 2.2 실행자(executable) 추가하기
* 이 코드를 빌드하기 위해서 먼저 CMakeLists.txt 파일을 열어보자. 
* 아래 코드를 find_package(rclcpp REQUIRED)  아래에 추가하자.
```cmake
add_executable(parameter_event_handler src/parameter_event_handler.cpp)
ament_target_dependencies(parameter_event_handler rclcpp)

install(TARGETS
  parameter_event_handler
  DESTINATION lib/${PROJECT_NAME}
)
```

### 3. 빌드 및 실행
* 빌드하기 전에 항상 workspace(ros2_ws)에서 rosdep 명령을 수행하는 것이 좋은 습관이다.
```
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```
* ros2_ws 로 이동하여 새 package를 빌드한다.
```
colcon build --packages-select cpp_parameter_event_handler
```

* 새 터미널을 열고 ros2_ws로 이동해서 setup 파일에 대해서 source를 수행한다.
```
. install/setup.bash
```

* 이제 해당 node를 실행한다.
```
ros2 run cpp_parameter_event_handler parameter_event_handler
```

* 이 node는 이제 실행되어 활성화되었다. 1개의 parameter를 가지고 있고 parameter가 업데이트될때마다 message를 출력한다. 
* 이를 테스트하기 위해서 새 터미널을 열고 ROS setup 파일을 source하고 아래 명령을 실행한다.
```
ros2 param set node_with_parameters an_int_param 43
```

* node를 실행한 터미널은 아래와 유사한 메시지를 출력하게 될 것이다.
```
[INFO] [1606950498.422461764] [node_with_parameters]: cb: Received an update to parameter "an_int_param" of type integer: "43"
```

* node 내에서 이전에 설정한 callback이 호출되고 새로운 업데이트된 값이 출력된다. 이제 터미널에서 Ctrl+C 키를 눌러서 parameter_event_handler 샘플을 종료시킬 수 있다.

### 3.1 다른 node의 parameters 변경 모니터링하기
* 다른 node의 parameter가 변경을 모니터링하기 위해서 ParameterEventHandler를 사용한다. 
* SampleNodeWithParameters class를 업데이트하면 다른 node 내에서의 parameter 변경을 모니터링해보자.
* parameter_blackboard 데모 어플리케이션을 사용하여 업데이트를 모니터링할 double parameter를 제공한다.

* 먼저 기존 코드에 아래 코드를 추가하여 생성자를 수정해보자.

```c++
// Now, add a callback to monitor any changes to the remote node's parameter. In this
// case, we supply the remote node name.
auto cb2 = [this](const rclcpp::Parameter & p) {
    RCLCPP_INFO(
      this->get_logger(), "cb2: Received an update to parameter \"%s\" of type: %s: \"%.02lf\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
auto remote_node_name = std::string("parameter_blackboard");
auto remote_param_name = std::string("a_double_param");
cb_handle2_ = param_subscriber_->add_parameter_callback(remote_param_name, cb2, remote_node_name);
```

* 다음으로 추가로 멤버 변수를 추가해보자. cb_handle2는 추가된 callback을 처리하기 위한 변수다.
```c++
private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle2_;  // Add this
};
```

* ros2_ws 빌드를 위해서 터미널로 가서 업데이트된 package를 빌드하자.
```
colcon build --packages-select cpp_parameter_event_handler
```

* setup 파일을 source 하자.
```
. install/setup.bash
```

* 이제 원격 parameter의 모니터링을 테스트하기 위해서 먼저 새로 생성한 parameter_event_handler 코드를 실행시켜보자.
```
ros2 run cpp_parameter_event_handler parameter_event_handler
```

* 다음으로 새로운 터미널에서 parameter_blackboard 데모 어플리케이션을 아래 명령으로 실행해보자.
```
ros2 run demo_nodes_cpp parameter_blackboard
```

* 마지막으로 3번째 터미널에서 parameter_blackboard node에서 parameter를 설정해보자. 
```
ros2 param set parameter_blackboard a_double_param 3.45
```

* 위 명령을 실행하면 parameter_event_handler window 내에서 아래와 같은 메시지가 출력된다. 메시지 내용은 parameter가 업데이트됨에 따라 callback 함수가 호출되었다는 내용이다.
```
[INFO] [1606952588.237531933] [node_with_parameters]: cb2: Received an update to parameter "a_double_param" of type: double: "3.45"
```

## 요약
* 하나의 parameter를 가지는 node를 생성했다.
* ParameterEventHandler class를 사용하여 parameter가 변경되는 것을 모니터링하는 callback을 설정하였다.
* 동일한 class를 이용하여 외부 node에 변경을 모니터링하는데 사용하였다.
* ParameterEventHandler는 parameter 변경을 모니터링하는데 편리한 방식을 제공한다. 따라서 업데이트된 값에 대해서 응답이 가능하다.
 