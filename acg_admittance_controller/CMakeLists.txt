cmake_minimum_required(VERSION 3.8)
project(acg_admittance_controller)

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

set(THIS_PACKAGE_INCLUDE_DEPENDS
    acg_common_libraries
    acg_control_msgs
    filters
    Eigen3
    generate_parameter_library
    geometry_msgs
    tf2_geometry_msgs
    motion_based_interaction_controller
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2_eigen
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

# Generate the parameter library
generate_parameter_library(acg_admittance_controller_parameters
                           src/acg_admittance_controller_parameters.yaml
)

# Shared library
add_library(acg_admittance_controller SHARED
            src/acg_admittance_controller.cpp
)

target_include_directories(
  acg_admittance_controller
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/acg_admittance_controller>
)

target_link_libraries(acg_admittance_controller acg_admittance_controller_parameters)

ament_target_dependencies(acg_admittance_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface acg_admittance_controller.xml)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  acg_admittance_controller
  PRIVATE "ACG_ADMITTANCE_CONTROLLER_BUILDING_DLL"
)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(TARGETS acg_admittance_controller acg_admittance_controller_parameters
        EXPORT export_acg_admittance_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

execute_process(COMMAND rm doc/acg_admittance_controller_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                ERROR_QUIET
)
execute_process(COMMAND generate_parameter_library_markdown --input_yaml
                        src/acg_admittance_controller_parameters.yaml --output_markdown_file
                        doc/acg_admittance_controller_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
