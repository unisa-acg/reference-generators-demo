cmake_minimum_required(VERSION 3.8)
project(end_effector_builder)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
)
