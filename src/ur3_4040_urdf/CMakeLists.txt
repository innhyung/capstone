cmake_minimum_required(VERSION 3.8)
project(ur3_4040_urdf)

# ROS 2 빌드 도구 찾기
find_package(ament_cmake REQUIRED)

# 설치할 폴더들 지정 (launch, meshes, urdf, config 등)
install(DIRECTORY
  launch
  meshes
  urdf
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Python 스크립트 (ros2 run 으로 실행)
install(PROGRAMS
  scripts/clean_map.py
  scripts/goal_relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()