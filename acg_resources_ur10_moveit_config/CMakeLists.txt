cmake_minimum_required(VERSION 3.8)
project(acg_resources_ur10_moveit_config)
find_package(ament_cmake REQUIRED)
find_package(ros_gz_bridge REQUIRED)

ament_package()

install(DIRECTORY config
                  launch
                  srdf
        DESTINATION share/${PROJECT_NAME}
)
