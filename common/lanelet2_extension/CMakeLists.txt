cmake_minimum_required(VERSION 2.8.11)
project(lanelet2_extension)

#
#- Modified to install add CARMA regulatory elements to build
#    - 2/19/2019
#    - Michael McConnell
#

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
  )
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)

find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

find_package(autoware_build_flags REQUIRED)

find_package(PROJ4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  amathutils_lib
  roscpp
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  autoware_lanelet2_msgs
  autoware_msgs
  geometry_msgs
  visualization_msgs
  roslint
  hardcoded_params
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lanelet2_extension_lib
  CATKIN_DEPENDS amathutils_lib roscpp lanelet2_core lanelet2_io lanelet2_maps lanelet2_projection lanelet2_routing lanelet2_traffic_rules lanelet2_validation autoware_lanelet2_msgs autoware_msgs geometry_msgs visualization_msgs hardcoded_params
)

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
  ${PROJ4_INCLUDE_DIRS}
)

add_definitions (${GeographicLib_DEFINITIONS})

add_library( lanelet2_extension_lib
  lib/autoware_osm_parser.cpp
  lib/autoware_traffic_light.cpp
  lib/local_frame_projector.cpp
  lib/message_conversion.cpp
  lib/mgrs_projector.cpp
  lib/query.cpp
  lib/utilities.cpp
  lib/visualization.cpp
  lib/CarmaUSTrafficRules.cpp
  lib/CarmaTrafficLight
  lib/PassingControlLine.cpp
  lib/StopRule.cpp
  lib/DigitalSpeedLimit.cpp
  lib/DigitalMinimumGap.cpp
  lib/RegionAccessRule.cpp
  lib/DirectionOfTravel.cpp
)

add_dependencies(lanelet2_extension_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lanelet2_extension_lib
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PROJ4_LIBRARIES}
)

add_executable(lanelet2_extension_sample src/sample_code.cpp)
add_dependencies(lanelet2_extension_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lanelet2_extension_sample
  ${catkin_LIBRARIES}
  lanelet2_extension_lib
)

add_executable(autoware_lanelet2_validation src/validation.cpp)
add_dependencies(autoware_lanelet2_validation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(autoware_lanelet2_validation
  ${catkin_LIBRARIES}
  ${PUGIXML_LIBRAREIS}
  lanelet2_extension_lib
)

install(TARGETS lanelet2_extension_lib lanelet2_extension_sample autoware_lanelet2_validation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
 roslint_add_test()
 find_package(rostest REQUIRED)
 add_rostest_gtest(message_conversion-test test/test_message_conversion.test test/src/test_message_conversion.cpp)
 target_link_libraries(message_conversion-test ${catkin_LIBRARIES} lanelet2_extension_lib)
 add_rostest_gtest(projector-test test/test_projector.test test/src/test_projector.cpp)
 target_link_libraries(projector-test ${catkin_LIBRARIES} lanelet2_extension_lib)
 add_rostest_gtest(query-test test/test_query.test test/src/test_query.cpp)
 target_link_libraries(query-test ${catkin_LIBRARIES} lanelet2_extension_lib)
 add_rostest_gtest(regulatory_elements-test test/test_regulatory_elements.test test/src/test_regulatory_elements.cpp)
 target_link_libraries(regulatory_elements-test ${catkin_LIBRARIES} lanelet2_extension_lib)
 add_rostest_gtest(utilities-test test/test_utilities.test test/src/test_utilities.cpp)
 target_link_libraries(utilities-test ${catkin_LIBRARIES} lanelet2_extension_lib)

 catkin_add_gmock(carma-additions-test
  test/src/CarmaTestsMain.cpp
  test/src/RegionAccessRuleTest.cpp
  test/src/PassingControlLineTest.cpp
  test/src/StopRuleTest.cpp
  test/src/DirectionOfTravelTest.cpp
  test/src/DigitalSpeedLimitTest.cpp
  test/src/DigitalMinimumGapTest.cpp
  test/src/CarmaUSTrafficRulesTest.cpp
  test/src/MapLoadingTest.cpp
  test/src/test_local_projector.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
 )

 target_link_libraries(carma-additions-test lanelet2_extension_lib ${catkin_LIBRARIES})
endif()
