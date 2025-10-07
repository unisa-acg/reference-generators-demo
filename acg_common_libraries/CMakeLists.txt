cmake_minimum_required(VERSION 3.8)
project(acg_common_libraries)

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

set(DIAGNOSTICS_LIB_INCLUDE_DEPENDS
    geometry_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
)

set(KINEMATICS_LIB_INCLUDE_DEPENDS
    acg_control_msgs
    kinematics_interface
    tf2_eigen
    tf2_geometry_msgs
)

set(INTERPOLATION_LIB_INCLUDE_DEPENDS
    acg_control_msgs
    geometry_msgs
    rclcpp
    tf2_eigen
)

set(MESSAGE_UTILITIES_LIB_INCLUDE_DEPENDS
    Eigen3
    acg_control_msgs
    geometry_msgs
    rclcpp
    tf2
)

set(URDF_UTILITIES_LIB_INCLUDE_DEPENDS
    Eigen3
    ros2_control_test_assets
    urdf
)

foreach(dependency IN
        ITEMS ${DIAGNOSTICS_LIB_INCLUDE_DEPENDS}
              ${INTERPOLATION_LIB_INCLUDE_DEPENDS}
              ${KINEMATICS_LIB_INCLUDE_DEPENDS}
              ${MESSAGE_UTILITIES_LIB_INCLUDE_DEPENDS}
              ${URDF_UTILITIES_LIB_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

# Header-only libraries must be represented as interface targets to correctly export their
# dependencies for dependents
add_library(diagnostics_lib INTERFACE)
add_library(interpolation_lib SHARED
            src/interpolation.cpp
)
add_library(kinematics_lib SHARED
            src/kinematics.cpp
)
add_library(message_utilities_lib SHARED
            src/message_utilities.cpp
)
add_library(urdf_utilities_lib SHARED
            src/urdf_utilities.cpp
)

target_compile_features(
  interpolation_lib
  PUBLIC cxx_std_17
)
target_compile_features(
  kinematics_lib
  PUBLIC cxx_std_17
)
target_compile_features(
  message_utilities_lib
  PUBLIC cxx_std_17
)
target_compile_features(
  urdf_utilities_lib
  PUBLIC cxx_std_17
)

target_include_directories(
  interpolation_lib
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/acg_common_libraries>
)
target_include_directories(
  kinematics_lib
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/acg_common_libraries>
)
target_include_directories(
  message_utilities_lib
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/acg_common_libraries>
)
target_include_directories(
  urdf_utilities_lib
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/acg_common_libraries>
)

ament_target_dependencies(
  diagnostics_lib
  INTERFACE
  ${DIAGNOSTICS_LIB_INCLUDE_DEPENDS}
)
ament_target_dependencies(interpolation_lib ${INTERPOLATION_LIB_INCLUDE_DEPENDS})
ament_target_dependencies(kinematics_lib ${KINEMATICS_LIB_INCLUDE_DEPENDS})
ament_target_dependencies(message_utilities_lib ${MESSAGE_UTILITIES_LIB_INCLUDE_DEPENDS})
ament_target_dependencies(urdf_utilities_lib ${URDF_UTILITIES_LIB_INCLUDE_DEPENDS})

install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY xacro
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS diagnostics_lib
        EXPORT export_diagnostics_lib
        INCLUDES
        DESTINATION include
)
install(TARGETS interpolation_lib
        EXPORT export_interpolation_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include
)
install(TARGETS kinematics_lib
        EXPORT export_kinematics_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include
)
install(TARGETS message_utilities_lib
        EXPORT export_message_utilities_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include
)
install(TARGETS urdf_utilities_lib
        EXPORT export_urdf_utilities_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include
)

if(BUILD_TESTING)
  # find test dependencies
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  set(THIS_TEST_PACKAGE_INCLUDE_DEPENDS
      pluginlib
      kinematics_interface
      kinematics_interface_kdl
  )

  foreach(dependency IN
          ITEMS ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS}
  )
    find_package(${dependency} REQUIRED)
  endforeach()

  set(TEST_NAME
      kinematics_test
  )

  # add tests
  ament_add_gtest_executable(
    ${TEST_NAME}
    test/test_kinematics.cpp
    test/test_interpolation.cpp
    test/test_message_utilities.cpp
    test/test_urdf_utilities.cpp
  )

  ament_target_dependencies(${TEST_NAME} ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})

  add_ros_test(test/test_kinematics.py)

  target_include_directories(
    ${TEST_NAME}
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(
    ${TEST_NAME}
    kinematics_lib
    interpolation_lib
    message_utilities_lib
    urdf_utilities_lib
  )

  install(TARGETS ${TEST_NAME}
          RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_export_include_directories(include)

ament_export_dependencies(
  ${DIAGNOSTICS_LIB_INCLUDE_DEPENDS}
  ${INTERPOLATION_LIB_INCLUDE_DEPENDS}
  ${KINEMATICS_LIB_INCLUDE_DEPENDS}
  ${MESSAGE_UTILITIES_LIB_INCLUDE_DEPENDS}
  ${URDF_UTILITIES_LIB_INCLUDE_DEPENDS}
)

ament_export_targets(export_diagnostics_lib HAS_LIBRARY_TARGET)
ament_export_targets(export_interpolation_lib HAS_LIBRARY_TARGET)
ament_export_targets(export_kinematics_lib HAS_LIBRARY_TARGET)
ament_export_targets(export_message_utilities_lib HAS_LIBRARY_TARGET)
ament_export_targets(export_urdf_utilities_lib HAS_LIBRARY_TARGET)
ament_package()
