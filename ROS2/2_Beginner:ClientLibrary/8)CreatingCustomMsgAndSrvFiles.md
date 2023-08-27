# [커스텀 msg와 srv 파일 생성하기](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. 새로운 package 생성하기
   2. 커스텀 정의 생성하기
   3. CMakeLists.txt
   4. package.xml
   5. tutorial_interfaces package 빌드하기
   6. msg와 srv 생성 확인하기
   7. 새로운 interfaces 테스트하기
5. 요약

## 목표
* 커스텀 interface 파일을 정의한다. : .msg와 .srv
* Python과 C++로 작성한 nodes에서 사용한다.

## 배경지식
* 이전 튜터리얼에서 topics, services에 대해서 배워보기 위해서 message와 service interface를 사용하였다.
* 우리가 사용했던 interfaces는 미리 정의된 것을 사용하였다. 

* 미리 정의된 interfaces를 사용하는 것을 추천한다!
  * 왜?? 
* 우리가 개발하는 시스템에 따라서 우리가 사용할 messages와 services를 정의해서 사용해야 하는 경우가 있다.
* 이 튜터리얼에서는 커스텀 interface 정의를 생성하는 가장 간단한 방법을 소개한다.

## 사전준비
* 기존 ROS2 workspace 
* 새로운 custom messages를 실행해보기 위해서 이전 pub/sub, service/client 튜터리얼에서 생성한 package를 사용한다.

## 실습
### 1. 새로운 package 생성하기
* 이 튜터리얼에서 커스텀 .msg와 .srv 파일을 우리 package 내에 생성한다.
* 다음으로 이를 다른 package 내에서 사용해본다.
* 생성하는 package와 사용하는 package는 동일한 workspace에 있어야만 한다.

* 이전 튜터리얼에서 작업한 pub/sub, service/client packages을 사용해서 진행한다.
* 따라서 이 2개 packages는 동일 workspace(ros2_ws/src) 내에 있어야 하며 아래 명령을 이용하여 interface를 생성하는 새로운 package를 생성한다.
```bash
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

* tutorial_interfaces는 새로운 package의 이름이다.
* 이 package는 CMake package이고 Python node에서는 현재는 .msg나 .srv 파일을 생성하는 방법이 없다.
* CMake package 내에서 커스텀 interface를 생성할 수 있고 다음으로 Python node 내에서 이를 사용할 수 있다.

* .msg와 .srv 파일은 각각 msg와 srv 디렉토리 내에 위치한다.
* ros2_ws/src/tutorial_interfaces 내부에 다음과 같이 2개 디렉토리를 생성하자. : 
```
mkdir msg

mkdir srv
```

### 2. 커스텀 정의 생성하기
### 2.1 msg 정의
* tutorial_interfaces/msg 디렉토리 내부에 Num.msg 파일을 생성하고 아래 내용을 붙여넣자. 자료구조는 다음과 같다.
```
int64 num
```
* tutorial_interfaces/msg 디렉토리 내부에 Sphere.msg 파일을 생성하고 아래 내용을 붙여넣자.
```
geometry_msgs/Point center
float64 radius
```
* 이 커스텀 message는 다른 message package의 message를(geometry_msgs/Point) 사용한다. (즉 message 내용을 복사하지 않고 원하는 message를 import하여 사용 가능)

### 2.2 srv 정의
* tutorial_interfaces/srv 디렉토리 내에, AddThreeInts.srv 라는 새로운 파일을 생성하고 아래와 같은 request/response 구조를 넣는다.
```
int64 a
int64 b
int64 c
---
int64 sum
```
* 3개의 정수 a, b, c를 request로 보내면 reponse로 합인 sum을 주고 받는 커스텀 service이다.

### 3. CMakeLists.txt
* interfaces를 특정 언어로 변환해야 해당 언어로 개발이 가능하다.
* CMakeLists.txt 파일에 다음과 같이 추가한다. (IDL : interface definition language)
```cmake
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```
### 4. package.xml
* interfaces는 특정 언어와 관련된 코드를 생성하기 위해서 rosidl_default_generators를 사용한다. 따라서 의존성을 추가해야 한다.
* <exec_depend> tag는 runtime이나 실행 stage 의존성을 지정하는데 사용한다. rosidl_interface_packages는 package가 속하는 의존성 group의 이름이다. <member_of_group> tag를 사용하여 선언한다.

* package.xml에 다음을 추가한다.
```xml
<depend>geometry_msgs</depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

### 5. tutorial_interfaces package 빌드하기
* 이제 custom interfaces package의 모든 부분이 갖춰졌다. 
* 이제 package를 빌드할 수 있다. 
* workspace(~/ros2_ws)에서 다음 명령을 실행하자.
```bash
colcon build --packages-select tutorial_interfaces
```
* 이제 이 interfaces는 다른 ROS2 packages가 참조할 수 있다.

### 6. msg와 srv 생성 확인
* 새로운 터미널을 열고 workspace(ros2_ws) 내에서 환경을 source한다.
```
. install/setup.bash
```

* ros2 interface show 명령을 사용하여 생성된 interface를 확인한다.
```
ros2 interface show tutorial_interfaces/msg/Num
```
* 다음과 같은 결과가 나온다.
```
int64 num
```
* 생성 interface 확인
```
ros2 interface show tutorial_interfaces/msg/Sphere
```
* 다음과 같은 결과가 나온다.
```
geometry_msgs/Point center
        float64 x
        float64 y
        float64 z
float64 radius
```

* 생성 interface 확인
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```

* 다음과 같은 결과가 나온다.
```
int64 a
int64 b
int64 c
---
int64 sum
```

### 7. 새로운 interface 테스트하기
* 이 단계에서는 이전 튜터리얼에서 생성한 packages를 사용하여 새로 생성한 interface를 사용해본다.
* 새로운 interfaces를 사용하려면 기존 package내에 있는 nodes, CMakeLists, package 파일을 수정해야 한다.

### 7.1 Num.msg  테스팅 하기 (pub/sub)
* 이전 튜터리얼의 publiser/subscriber package에 대해서 몇 가지 수정된 것들로 action에서 Num.msg을 볼 수 있다. 
* 표준 string msg를 수치 형태로 변경할려고 하기 때문에 결과가 약간 다를 수 있다.
* Publisher:
```c++
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"                                            // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);  // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::Num();                                   // CHANGE
    message.num = this->count_++;                                                     // CHANGE
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;             // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
* Subscriber :
```c++
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"                                       // CHANGE

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(    // CHANGE
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Num & msg) const  // CHANGE
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");     // CHANGE
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;  // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

* CMakeLists.txt
```cmake
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                      # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)    # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)  # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```
* package.xml
```xml
<depend>tutorial_interfaces</depend>
```
* 위 내용들을 수정한 후에 저장하고 package 빌드하기
```
colcon build --packages-select cpp_pubsub
```

* 2개의 새 터미널을 열고, ros2_ws 를 각각 source하고 실행한다.
```bash
ros2 run cpp_pubsub talker
```

```bash
ros2 run cpp_pubsub listener
```

* Num.msg는 integer만 사용하므로 talker는 정수 값만 publish해야 한다. (이전에는 string을 publish하였음)
```
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
```

### 7.2 AddThreeInts.srv 테스팅하기 (service/client)
* 이전 튜터리얼의 service/client package에 대해서 몇 가지 수정된 것들로 action에서 AddThreeInts.srv를 보자. 
* 원래는 2개 정수를 request했었는데 이번에는 3개 정수를 request하므로 결과가 달라진다.

* Service:
```c++
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"                                        // CHANGE

#include <memory>

void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     // CHANGE
          std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  // CHANGE
{
  response->sum = request->a + request->b + request->c;                                      // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  // CHANGE
                request->a, request->b, request->c);                                         // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   // CHANGE

  rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =               // CHANGE
    node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

* Client :
```c++
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"                                       // CHANGE

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) { // CHANGE
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");      // CHANGE
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");  // CHANGE
  rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =                // CHANGE
    node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");          // CHANGE

  auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();       // CHANGE
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);
  request->c = atoll(argv[3]);                                                              // CHANGE

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}
```

* CMakeLists.txt
```cmake
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)         # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)                      # CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      # CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

* package.xml:
```xml
<depend>tutorial_interfaces</depend>
```

* 위 내용들을 수정한 후에 저장하고 package 빌드하기
```
colcon build --packages-select cpp_srvcli
```

* 2개의 새 터미널을 열고, ros2_ws 를 각각 source하고 실행한다.
```bash
ros2 run cpp_srvcli server
```

```bash
ros2 run cpp_srvcli client 2 3 1
```

## 요약
* 이 튜터리얼에서 커스텀 interfaces를 우리 package 내에 생성하는 방법을 배웠다.
* 다른 package 내에서 생성한 커스텀 interfaces를 사용하는 방법을 배웠다.
* 간단한 interface 생성 및 사용 방법이다.