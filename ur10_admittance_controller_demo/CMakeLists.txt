cmake_minimum_required(VERSION 3.8)
project(ur10_admittance_controller_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
