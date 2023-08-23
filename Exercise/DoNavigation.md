# Navigation 하기
* 목차
  1. 사전에 지도를 만들어 놓았다.
  1. 지도 upload하기
  1. jarabot 접속하기
  1. jarabot에서 실행
  1. PC에서 실행
  1. Jarabot 이동시키기

## 지도 Upload하기

## jaraot 접속하기
* SSH 접속
  * ID, passwd로 접속

## jarabot에서 실행
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jarabot_node test.launch.py
```
* 새 터미널에서
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jarabot_navigation2 navigation2.launch.py map_file:=${HOME}/maps/map.yaml
```

## PC에서 실행
* rviz2 실행
```bash
ros2 run rviz2 rviz2 

# 혹은 rviz 설정파일 불러오기
ros2 run rviz2 rviz2 ~/ros2_ws/src/jarabot/jarabot_cartographer/rviz/jarabot_cartographer.rviz
```

## 지도를 로드하여 확인
* map_server.launch.py 파일에서 map 파일 위치 수정(!)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jarabot_node map_server.launch.py
```
* rviz2 설정(map 확인을 위해서)
  * Topic : /map
    * History : Keep Last
    * Reliability : Reliable
    * Durability : Transient Local   

## Jarabot 이동시키기
* 2D Pose Estimate 수행
* 2D Nav Goal 수행

