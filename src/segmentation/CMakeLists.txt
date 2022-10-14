cmake_minimum_required(VERSION 3.8)
project(segmentation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package( PCL 1.8 REQUIRED )
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pc_subscriber src/segmentation.cpp)
ament_target_dependencies(pc_subscriber pcl_conversions rclcpp sensor_msgs visualization_msgs)

add_executable(normal_pub src/normal.cpp)
ament_target_dependencies(normal_pub pcl_conversions rclcpp sensor_msgs visualization_msgs)

add_executable(stitch src/stitch.cpp)
ament_target_dependencies(stitch pcl_conversions rclcpp sensor_msgs tf2 tf2_ros)


if( PCL_FOUND )

  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )

  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries(pc_subscriber ${PCL_LIBRARIES} )
  target_link_libraries(normal_pub ${PCL_LIBRARIES} )
  target_link_libraries(stitch ${PCL_LIBRARIES} )

endif()


install(DIRECTORY config launch worlds urdf
  DESTINATION share/${PROJECT_NAME}) 

install(TARGETS
  pc_subscriber
  normal_pub
  stitch
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()  
  
ament_package()
