cmake_minimum_required(VERSION 3.16)
project(reference_generator
        LANGUAGES CXX
)

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

set(REFERENCE_GENERATOR_INCLUDE_DEPENDS
    controller_interface
    generate_parameter_library
    geometry_msgs
    pluginlib
    kinematics_interface
    realtime_tools
    acg_hardware_interface_facade
    acg_common_libraries
)

set(JOINT_SPACE_REFERENCE_GENERATOR_INCLUDE_DEPENDS
    generate_parameter_library
    kinematics_interface
    pluginlib
    realtime_tools
    acg_control_msgs
    rclcpp_action
    acg_hardware_interface_facade
    acg_common_libraries
)

set(TASK_SPACE_REFERENCE_GENERATOR_INCLUDE_DEPENDS
    generate_parameter_library
    geometry_msgs
    kinematics_interface
    pluginlib
    realtime_tools
    tf2_geometry_msgs
    acg_control_msgs
    rclcpp_action
    acg_hardware_interface_facade
    acg_common_libraries
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN
        ITEMS ${REFERENCE_GENERATOR_INCLUDE_DEPENDS}
              ${JOINT_SPACE_REFERENCE_GENERATOR_INCLUDE_DEPENDS}
              ${TASK_SPACE_REFERENCE_GENERATOR_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(joint_space_reference_generator_parameters
                           src/joint_space_reference_generator_parameters.yaml
)

generate_parameter_library(task_space_reference_generator_parameters
                           src/task_space_reference_generator_parameters.yaml
)

generate_parameter_library(reference_generator_parameters src/reference_generator_parameters.yaml)

add_library(joint_space_reference_generator SHARED
            src/joint_space_reference_generator.cpp
)
add_library(task_space_reference_generator SHARED
            src/task_space_reference_generator.cpp
)
add_library(reference_generator SHARED
            src/reference_generator.cpp
)
target_compile_features(
  joint_space_reference_generator
  PUBLIC cxx_std_17
)
target_compile_features(
  task_space_reference_generator
  PUBLIC cxx_std_17
)
target_compile_features(
  reference_generator
  PUBLIC cxx_std_17
)
target_include_directories(
  joint_space_reference_generator
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/reference_generator>
)
target_include_directories(
  task_space_reference_generator
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/reference_generator>
)
target_include_directories(
  reference_generator
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/reference_generator>
)

target_link_libraries(
  reference_generator
  PUBLIC reference_generator_parameters
)
target_link_libraries(
  joint_space_reference_generator
  PUBLIC joint_space_reference_generator_parameters
  PRIVATE reference_generator
)
target_link_libraries(
  task_space_reference_generator
  PUBLIC task_space_reference_generator_parameters
  PRIVATE reference_generator
)

ament_target_dependencies(
  joint_space_reference_generator
  PUBLIC
  ${JOINT_SPACE_REFERENCE_GENERATOR_INCLUDE_DEPENDS}
)
ament_target_dependencies(
  task_space_reference_generator
  PUBLIC
  ${TASK_SPACE_REFERENCE_GENERATOR_INCLUDE_DEPENDS}
)
ament_target_dependencies(
  reference_generator
  PUBLIC
  ${REFERENCE_GENERATOR_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  joint_space_reference_generator
  PRIVATE "JOINT_SPACE_REFERENCE_GENERATOR_BUILDING_DLL"
)
pluginlib_export_plugin_description_file(controller_interface joint_space_reference_generator.xml)

target_compile_definitions(
  task_space_reference_generator
  PRIVATE "TASK_SPACE_REFERENCE_GENERATOR_BUILDING_DLL"
)
pluginlib_export_plugin_description_file(controller_interface task_space_reference_generator.xml)

install(DIRECTORY include/
        DESTINATION include/reference_generator
)

install(TARGETS reference_generator reference_generator_parameters
        EXPORT export_reference_generator
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(TARGETS joint_space_reference_generator joint_space_reference_generator_parameters
        EXPORT export_joint_space_reference_generator
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(TARGETS task_space_reference_generator task_space_reference_generator_parameters
        EXPORT export_task_space_reference_generator
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

set(REFERENCE_GENERATORS
    reference_generator
    joint_space_reference_generator
    task_space_reference_generator
)
foreach(controller IN
        ITEMS ${REFERENCE_GENERATORS}
)
  execute_process(COMMAND rm doc/${controller}_parameters.md
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                  ERROR_QUIET
  )
  execute_process(COMMAND generate_parameter_library_markdown --input_yaml
                          src/${controller}_parameters.yaml --output_markdown_file
                          doc/${controller}_parameters.md
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )
endforeach()

ament_export_targets(export_joint_space_reference_generator HAS_LIBRARY_TARGET)
ament_export_targets(export_task_space_reference_generator HAS_LIBRARY_TARGET)
ament_export_targets(export_reference_generator HAS_LIBRARY_TARGET)

ament_export_dependencies(${JOINT_SPACE_REFERENCE_GENERATOR_INCLUDE_DEPENDS})
ament_export_dependencies(${TASK_SPACE_REFERENCE_GENERATOR_INCLUDE_DEPENDS})
ament_export_dependencies(${REFERENCE_GENERATOR_INCLUDE_DEPENDS})
ament_package()
