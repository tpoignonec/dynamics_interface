cmake_minimum_required(VERSION 3.16)
project(dynamics_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  kinematics_interface
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

# Create interface library for kinematics base class
add_library(dynamics_interface SHARED src/dynamics_interface.cpp)
target_compile_features(dynamics_interface PUBLIC cxx_std_17)
target_include_directories(dynamics_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/dynamics_interface>
)
target_link_libraries(dynamics_interface PUBLIC
  Eigen3::Eigen
)
ament_target_dependencies(dynamics_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dynamics_interface PUBLIC "DYNAMICS_INTERFACE_BUILDING_DLL")

install(
  DIRECTORY include/
  DESTINATION include/dynamics_interface
)
install(
  TARGETS dynamics_interface
  EXPORT export_dynamics_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_dynamics_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
