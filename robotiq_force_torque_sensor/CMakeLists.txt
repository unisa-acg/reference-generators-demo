cmake_minimum_required(VERSION 3.8)
project(robotiq_force_torque_sensor)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robotiq_ft_sensor_description REQUIRED)
find_package(robotiq_ft_sensor_hardware REQUIRED)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

# Package
ament_package()
