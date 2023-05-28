cmake_minimum_required(VERSION 3.8)
project(ndt_matching_2d)
set(COMP_NAME "NDT_MATCHING_2D")
string(TOLOWER ${COMP_NAME} COMP_NAME_LOWER)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(pcl_conversions)
ament_export_dependencies(pcl_apps_msgs)

# Create the component target
ament_auto_add_library(${COMP_NAME_LOWER}_component SHARED
    src/${COMP_NAME_LOWER}_component.cpp
)
target_compile_definitions(${COMP_NAME_LOWER}_component
    PRIVATE ${PROJECT_NAME}_BUILDING_DLL)
target_link_libraries(${COMP_NAME_LOWER}_component
  ${PCL_LIBRARIES})
rclcpp_components_register_nodes(${COMP_NAME_LOWER}_component
    "${PROJECT_NAME}::NdtMatching2dComponent")

# Create the executor target
ament_auto_add_executable(${COMP_NAME_LOWER}_node
    NO_TARGET_LINK_LIBRARIES
    src/${COMP_NAME_LOWER}_node.cpp
)
target_link_libraries(${COMP_NAME_LOWER}_node
    ${COMP_NAME_LOWER}_component
    ${PCL_LIBRARIES}
    )

ament_auto_package()

