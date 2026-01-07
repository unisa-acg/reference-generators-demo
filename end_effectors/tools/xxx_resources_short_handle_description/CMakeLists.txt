cmake_minimum_required(VERSION 3.8)
project(xxx_resources_short_handle_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config
                  launch
                  meshes
                  urdf
        DESTINATION share/${PROJECT_NAME}
)
