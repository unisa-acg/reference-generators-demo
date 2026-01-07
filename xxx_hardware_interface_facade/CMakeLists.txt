cmake_minimum_required(VERSION 3.16)
project(xxx_hardware_interface_facade
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

set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    xxx_control_msgs
    xxx_controller_interface
    controller_interface
    geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(launch_testing_ament_cmake)
foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(hardware_interface_facade SHARED
            src/command_interface_handler.cpp
            src/state_reader.cpp
            src/command_writer.cpp
            src/reference_reader.cpp
            src/force_torque_sensor_reader.cpp
)

target_compile_features(
  hardware_interface_facade
  PUBLIC cxx_std_17
)

target_include_directories(
  hardware_interface_facade
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/xxx_hardware_interface_facade>
)

target_link_libraries(hardware_interface_facade)

ament_target_dependencies(
  hardware_interface_facade
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(DIRECTORY include/
        DESTINATION include/xxx_hardware_interface_facade
)

install(TARGETS hardware_interface_facade
        EXPORT export_hardware_interface_facade
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_export_targets(export_hardware_interface_facade HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# tests
if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_NAME
      ${PROJECT_NAME}_test
  )

  ament_add_gtest(${TEST_NAME}
                  test/test_state_reader.cpp
                  test/test_command_interface_handler.cpp
                  test/test_command_writer.cpp
                  test/test_reference_reader.cpp
                  test/test_force_torque_sensor_reader.cpp
  )

  target_include_directories(
    ${TEST_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(${TEST_NAME} hardware_interface_facade)

  install(TARGETS ${TEST_NAME}
          RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_package()
