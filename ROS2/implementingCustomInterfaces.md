# 커스텀 interfaces 구현하기
1. 개요
2. 실습
   1. package 생성하기
   2. msg 파일 생성하기
   3. 동일 package에서 interface 사용하기
   4. 해보기
## 1. 개요
* 커스텀 interfaces를 구현하는 방법 (내 package에서 커스텀 interface를 선언하고 사용하기)
  * custom interfaces를 특정 package내에서 구현하는 것을 추천하지만
  * 하나의 package내에서 선언, 생성, 사용하는 것이 가끔은 편리할 수 있다! (추천하지는 않음)
## 2. 실습
### 2-1 package 생성하기
* more_interfaces package 생성하고 이 package 내부에 msg 디렉토리 만들기
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake more_interfaces
mkdir more_interfaces/msg
```

### 2-2 msg 파일 생성하기
* more_interfaces/msg 내부에 AddressBook.msg 파일 생성하고 아래 코드 붙여넣기
```
uint8 PHONE_TYPE_HOME=0
uint8 PHONE_TYPE_WORK=1
uint8 PHONE_TYPE_MOBILE=2

string first_name
string last_name
string phone_number
uint8 phone_type
```

### 2-2-1 msg 파일 빌드하기
* package.xml 열고 아래 코드 추가하기
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

* CMakeLists.txt 열고 아래 코드 추가하기
```cmake
find_package(rosidl_default_generators REQUIRED)
```

* 생성할 messages의 목록을 선언 (CMakeLists.txt)
```
set(msg_files
  "msg/AddressBook.msg"
)
```

* message 생성
```
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
```

* runtime 의존성
```
ament_export_dependencies(rosidl_default_runtime)
```

### 2-3 동일 package에서 interface 사용하기
* more_interfaces/src/publish_address_book.cpp 파일 생성하고 아래 내용 복사하기
```c++
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher()
  : Node("address_book_publisher")
  {
    address_book_publisher_ =
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

    auto publish_msg = [this]() -> void {
        auto message = more_interfaces::msg::AddressBook();

        message.first_name = "John";
        message.last_name = "Doe";
        message.phone_number = "1234567890";
        message.phone_type = message.PHONE_TYPE_MOBILE;

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();

  return 0;
}
```

### 2-3-2 publisher 빌드하기
* CMakeLists.txt 수정(새로운 target 생성)
```cmake
find_package(rclcpp REQUIRED)

add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)

install(TARGETS
    publish_address_book
  DESTINATION lib/${PROJECT_NAME})
```

### 2-3-3 interface에 대한 Link
* CMakeLists.txt 수정(동일 package내에서 생성된 message 사용하기 위해 추가)
```cmake
rosidl_target_interfaces(publish_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
```

### 2-4 해보기
* 빌드하기
```bash
cd ~/ros2_ws
colcon build --packages-up-to more_interfaces
```

* publisher 실행하기
```bash
source install/local_setup.bash
ros2 run more_interfaces publish_address_book
```

* address_book topic으로 publish되는 message 확인하기 (echo 명령 사용)
```bash
source install/setup.bash
ros2 topic echo /address_book
```

