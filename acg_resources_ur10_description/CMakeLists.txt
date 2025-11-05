cmake_minimum_required(VERSION 3.8)
project(acg_resources_ur10_description)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config
                  launch
                  urdf
        DESTINATION share/${PROJECT_NAME}
)
