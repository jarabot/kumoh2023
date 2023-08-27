# [여러 nodes를 하나의 단일 process로 구성하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html)
1. 목표
2. 배경지식
3. demos 실행하기
   1. discover 가능한 components
   2. ROS2 Pub/Sub을 사용한 run-time composition
   3. ROS2 server/client 사용한 run-time composition
   4. ROS services를 사용한 compile-time composition
   5. dlopen을 사용한 run-time composition
   6. launch actions를 사용한 composition 
4. 고급 주제
   1. components unload하기
   2. container name과 namespace remapping
   3. component name과 namespaces를 remapping
   4. parameter values를 components로 전달하기
   5. 추가 arguments를 components로 전달하기
5. 공유 library형태로 nodes 구성

## 목표
* 여러 개의 nodes를 하나의 process로 구성하기

## 배경지식
* [Composition 개념 이해](https://docs.ros.org/en/humble/Concepts/About-Composition.html)

## demos 실행하기
* rclcpp_components, ros2component, composition package에 있는 실행자(executables)를 사용하여 demo 진행
* 아래 명령을 따라서 실행할 수 있다.

### 1. discover 가능한 components
* workspace에서 사용 가능한 등록된 components인지 보기 위한 명령은 아래와 같다.
```bash
ros2 component types
```

* 위 명령 수행 결과로 사용 가능한 모든 components의 목록을 보여준다.
```
(... components of other packages here)
composition
  composition::Talker
  composition::Listener
  composition::NodeLikeListener
  composition::Server
  composition::Client
(... components of other packages here)
```

### 2. ROS Pub/Sub을 이용한 run-time composition
* 첫번째 shell에서 component container를 구동시킨다.
```
ros2 run rclcpp_components component_container
```
* 두번째 터미널 열고 위에 container가 실행 중인지 확인하는 명령 실행
```
ros2 component list
```
* component name이 결과로 나온다.
```
/ComponentManager
```

* 두번째 터미널에서 talker component를 로드하기 ([talker 소스코드](https://github.com/ros2/demos/blob/humble/composition/src/talker_component.cpp))
```
ros2 component load /ComponentManager composition composition::Talker
```
* 명령의 결과로 로드된 component의 ID와 node 이름을 반환한다.
```
Loaded component 1 into '/ComponentManager' container node as '/talker'
```

* 이제 첫번째 터미널에서 주기적으로 publish message와 로드된 message를 보여준다.

* 두번째 터미널에서 listener component를 로드하기 위해서 또다른 command를 실행한다. ([listener 소스코드](https://github.com/ros2/demos/blob/humble/composition/src/listener_component.cpp))
```
ros2 component load /ComponentManager composition composition::Listener
```
* 터미널은 다음과 같은 결과를 보여준다.
```
Loaded component 2 into '/ComponentManager' container node as '/listener'
```

* ros2 명령을 이용하여 container의 상태를 확인할 수 있다.
```
ros2 component list
```
* 주의 : ros2 component 명령 실행시 error 발생하는 경우 ros2 데몬 재실행
```
ros2 daemon stop
ros2 daemon start
```

* 다음과 같은 결과가 나온다.
```
/ComponentManager
   1  /talker
   2  /listener
```

* 자 이제 첫번째 터미널은 매번 수신한 message에 대해서 매번 출력한다.

### 3. ROS2 server/client를 사용한 run-time composition
* 위에 예제와 비슷한다.
* 첫번째 터미널에서 실행
```
ros2 run rclcpp_components component_container
```

* 두번째 터미널에서 실행 ([server](https://github.com/ros2/demos/blob/humble/composition/src/server_component.cpp), [client](https://github.com/ros2/demos/blob/humble/composition/src/client_component.cpp))
```
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```

* 이 경우 client는 server에게 request를 보낸다. server는 이 request를 처리하고 response를 client에게 보낸다. client를 이를 수신하여 출력한다.

### ROS2 service를 사용한 compile-time composition
* 이 데모는 동일한 shared libraries를 사용하여 컴파일하는 것을 보여준다. 여러 components를 실행시킬 수 있는 하나의 실행자를 컴파일한다.
* 이 실행자는 위에서처럼 모두 4개 components를 포함하고 있다. talker, listener, server, client

* 이 터미널에서 아래와 같이 실행한다.
```
ros2 run composition manual_composition
```

* 2개 쌍(talker/listener, server/client)이 주기적으로 messages를 보여준다.
* Note
  * 수동으로 compose한 components는 ros2 component list 명령에서 나타나지 않는다. (수동으로 compose하면 등록이 안되어서 보이지 않음)

### 5. dlopen를 사용한 run-time composition
* 이 데모는 아래에 대한 대안
  * generic container process를 생성한 run-time composition
  * ROS2 interface를 사용하지 않고 load하기 위해서 명시적으로 libraries 전달
* 이 process는 각 library를 열고, 그 library 내에 있는 각 “rclcpp::Node” class의 인스턴스를 생성한다.
```
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```
* 이제 해당 터미널은 각 send/receive message를 출력한다.
* Note
  * ldopen-composed components는 ros2 component list 명령으로 출력되지 않는다.

### 6. launch actions을 사용한 composition
* command line 도구를 이용하면 component 설정을 디버깅하고 진단하기 편리하다.
* 한번에 여러 components 시작시키기 편리하다.
* 이런 동작을 자동화하기 위해서 ros2 launch 명령을 사용할 수 있다.
```
ros2 launch composition composition_demo.launch.py
```

## 4. 고급 주제
* components에 대한 기본적인 동작을 지금까지 알아보았다. 이제는 좀더 고급 주제에 대해서 알아보자.

###   4.1. components unload하기
* 첫번째 터미널에서 아래 명령으로 component container를 구동시킨다.
```
ros2 run rclcpp_components component_container
```

* ros2 명령을 이용해서 container가 실행 중인지 확인한다.
```
ros2 component list
```

* 결과로 해당 component의 이름이 나와야 한다.
```
/ComponentManager
```
* 두번째 터미널에서 위에서 했던 것과 같이 talker와 listener 쌍을 load 한다.
```
ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener
```
* component container에서 특정 node를 unload하기 위해서 ID를 사용한다.
```
ros2 component unload /ComponentManager 1 2
```
* 결과로 다음과 같이 출력된다.
```
Unloaded component 1 from '/ComponentManager' container
Unloaded component 2 from '/ComponentManager' container
```

* 첫번째 터미널에서 talker와 listener에서 message 출력이 중단되는지 확인하자.

###   4.2. container name과 namespace remapping
* component manager name과 namespace는 표준 command line 인자를 통해서 remapping될 수 있다.
```
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns
```

* 두번째 터미널에서 components는 업데이트되니 container name을 사용하여 load시킬 수 있다.
```
ros2 component load /ns/MyContainer composition composition::Listener
```

* Note
  * container의 namespace remappings은 로드된 components에 영향을 미치지 않는다.

###   4.3. component name과 namespaces를 remapping
* component names과 namespace는 load 명령의 인자를 통해서 조정 가능하다.
* 첫번째 터미널에서 component container를 시작시킨다 :
```
ros2 run rclcpp_components component_container
```
* names과 namespaces를 remap하는 방법을 보여주는 몇가지 예제를 살펴보자.
  * node name을 remap :
     ```
     ros2 component load /ComponentManager composition composition::Talker --node-name talker2
     ```
  * namespace remap :
     ```
     ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
     ```
  * node names과 namespaces 모두 remap
     ```
     ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2
     ```
* 이제 ros2 명령을 이용해보자.
```
ros2 component list
```

* 콘솔에 관련 내용이 출력된다.
```
/ComponentManager
   1  /talker2
   2  /ns/talker
   3  /ns2/talker3
```
* container의 namespace remapping은 로드된 components에는 영향을 미치지 않는다.

###   4.4. parameter values를 components로 전달하기
* ros2 component load 명령을 사용하면 임의의 parameters를 node에 전달할 수 있다. 이 기능은 다음과 같이 사용할 수 있다.
```
ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true
```

###   4.5. 추가 arguments를 components로 전달하기
* ros2 component load 명령은 특정 옵션을 component manager에게 전달할 수 있다. 
* 지원되는 유일한 command line 옵션은 intra-process communication을 통해서 node 인스턴스를 생성하는 것이다.
* 이 기능은 아래 명령으로 실행할 수 있다.
```
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true
```

## 공유 library형태로 구성가능한 nodes
* compose로 사용할 수 있는 node를 package에서 shared library로 외주에 제공하고 다른 package에서 이 node를 사용하고자 한다면, CMake 파일에 코드를 추가해야한다. CMake 파일에 실제 target을 import시켜야 한다. (즉 사용하고자 하는 node 정보를 CMake에 넣어줘야 shared lib를 링크하여 빌드가 된다.)

* 빌드가 완료되면 생성된 파일을 install하고 외부에 export시킨다.
* [실제 예제](https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602)
