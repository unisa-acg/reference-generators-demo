cmake_minimum_required(VERSION 3.8)
project(acg_controller_interface)

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
    hardware_interface
    rclcpp
    geometry_msgs
    controller_interface
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

# Header-only libraries must be represented as interface targets to correctly export their
# dependencies for dependents
add_library(semantic_components_lib INTERFACE)

ament_target_dependencies(
  semantic_components_lib
  INTERFACE
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(DIRECTORY include/
        DESTINATION include
)

install(TARGETS semantic_components_lib
        EXPORT export_semantic_components_lib
        INCLUDES
        DESTINATION include
)

if(BUILD_TESTING)
  # find test dependencies
  find_package(ament_cmake_gtest REQUIRED)

  set(THIS_TEST_PACKAGE_INCLUDE_DEPENDS
      geometry_msgs hardware_interface
  )

  foreach(dependency IN
          ITEMS ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS}
  )
    find_package(${dependency} REQUIRED)
  endforeach()

  set(TEST_NAME
      ${PROJECT_NAME}_test
  )

  # add tests
  ament_add_gtest(${TEST_NAME}
                  test/test_semantic_component_command_interface.cpp
                  test/test_pose_command.cpp
                  test/test_twist_command.cpp
                  test/test_accel_command.cpp
                  test/test_wrench_command.cpp
                  test/test_wrench_derivative_command.cpp
                  test/test_joint_position_command.cpp
                  test/test_joint_velocity_command.cpp
                  test/test_joint_acceleration_command.cpp
                  test/test_joint_effort_command.cpp
                  test/test_position_encoder.cpp
                  test/test_velocity_encoder.cpp
                  test/test_accelerometer.cpp
                  test/test_effort_sensor.cpp
                  test/test_twist_sensor.cpp
  )

  ament_target_dependencies(${TEST_NAME} ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})

  target_link_libraries(${TEST_NAME} semantic_components_lib)

  target_include_directories(
    ${TEST_NAME}
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )

  install(TARGETS ${TEST_NAME}
          RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_semantic_components_lib HAS_LIBRARY_TARGET)

ament_package()
