cmake_minimum_required(VERSION 3.16)
project(dynamics_interface_fd LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  kdl_parser
  dynamics_interface_kdl
  pluginlib
  tf2_eigen_kdl
  realtime_tools
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(
  dynamics_interface_fd
  SHARED
  src/dynamics_interface_fd.cpp
)
target_include_directories(dynamics_interface_fd PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/dynamics_interface_fd>
)
target_compile_features(dynamics_interface_fd PUBLIC cxx_std_17)
ament_target_dependencies(dynamics_interface_fd PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(dynamics_interface_fd PUBLIC
  Eigen3::Eigen
)

pluginlib_export_plugin_description_file(dynamics_interface dynamics_interface_fd.xml)
pluginlib_export_plugin_description_file(kinematics_interface kinematics_dynamics_interface_fd.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_dynamics_interface_fd
    test/test_dynamics_interface_fd.cpp
  )
  target_link_libraries(test_dynamics_interface_fd dynamics_interface_fd)

  # TODO: Use target_link_libraries once ros2_control_test_assets'
  #       CMake include export is fixed
  ament_target_dependencies(test_dynamics_interface_fd ros2_control_test_assets)
endif()

install(
  DIRECTORY include/
  DESTINATION include/dynamics_interface_fd
)
install(
  TARGETS dynamics_interface_fd
  EXPORT export_dynamics_interface_fd
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_dynamics_interface_fd HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
