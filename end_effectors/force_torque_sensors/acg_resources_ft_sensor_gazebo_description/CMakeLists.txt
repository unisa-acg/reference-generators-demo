cmake_minimum_required(VERSION 3.8)
project(acg_resources_ft_sensor_gazebo_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY urdf world
        DESTINATION share/${PROJECT_NAME}
)
