cmake_minimum_required(VERSION 3.8)
project(makerspet_loki)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)


install(DIRECTORY rviz urdf sdf config map
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS script/clone_robot_description.sh script/urdf2sdf.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
