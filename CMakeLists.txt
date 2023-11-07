cmake_minimum_required(VERSION 3.8)
project(makerspet_loki)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)


install(DIRECTORY rviz urdf sdf config sdf/${PROJECT_NAME}/mesh
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
