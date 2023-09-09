cmake_minimum_required(VERSION 3.8)
project(joint_to_cartesian_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  realtime_tools
  kdl_parser
  geometry_msgs
  tf2_kdl
  tf2
  urdf
  pluginlib
  generate_parameter_library
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

generate_parameter_library(
  joint_to_cartesian_controller_parameters
  src/joint_to_cartesian_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/joint_to_cartesian_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
  joint_to_cartesian_controller_parameters
)
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
