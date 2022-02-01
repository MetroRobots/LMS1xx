cmake_minimum_required(VERSION 3.5)
project(lms1xx)
find_package(ament_cmake REQUIRED)

# Build ROS-independent library.
# find_package(console_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# include_directories(include ${console_bridge_INCLUDE_DIRS})

# ROS Dependent

# include_directories(include)
# add_library(LMS1xx src/LMS1xx.cpp)
# ament_target_dependencies(LMS1xx ${console_bridge_LIBRARIES})
# add_executable(LMS1xx_node src/LMS1xx_node.cpp)
# ament_target_dependencies(LMS1xx_node LMS1xx "launch" "sensor_msgs" "launch_ros")

# install(TARGETS LMS1xx LMS1xx_node
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY meshes launch urdf
  DESTINATION share/${PROJECT_NAME}
)

#install(PROGRAMS scripts/find_sick scripts/set_sick_ip
#   DESTINATION lib/${PROJECT_NAME}
#)

#ament_export_dependencies(rclcpp)

if (BUILD_TESTING)
  #roslint_add_test()

  # find_package(roslaunch REQUIRED)

  # find_package(roslint REQUIRED)
  # target_link_libraries(test_buffer ${catkin_LIBRARIES})
  # roslint_cpp()
  # ament_add_gtest(test_buffer test/test_buffer.cpp)
  # roslaunch_add_file_check(launch/LMS1xx.launch)
endif()
ament_package()
