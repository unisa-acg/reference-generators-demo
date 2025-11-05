cmake_minimum_required(VERSION 3.8)
project(ur10_cartesian_controller_demo)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(ros_gz_bridge REQUIRED)

ament_package()

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)
