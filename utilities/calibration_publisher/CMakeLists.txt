cmake_minimum_required(VERSION 2.8.3)
project(calibration_publisher)

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        autoware_msgs
        tf2
        tf2_ros
        geometry_msgs
        )

find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        autoware_msgs
        tf2
        tf2_ros
        geometry_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(calibration_publisher
        src/calibration_publisher.cpp
        )

add_dependencies(calibration_publisher
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(calibration_publisher
        ${catkin_LIBRARIES}
        )

install(TARGETS calibration_publisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
