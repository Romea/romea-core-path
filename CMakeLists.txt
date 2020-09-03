cmake_minimum_required(VERSION 2.8.3)
project(romea_path)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17 -O3")

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS romea_localisation)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(GSL REQUIRED)
find_package(BLAS REQUIRED)
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

###########
## Build ##
###########

include_directories(
    include
    SYSTEM ${catkin_INCLUDE_DIRS}
    SYSTEM ${EIGEN3_INCLUDE_DIRS}
    )

add_library(romea_path
    src/Path2D.cpp
    src/PathCurve2D.cpp
    src/PathFrenetPose2D.cpp
    src/PathMatchedPoint2D.cpp
    src/PathMatching2D.cpp
    src/PathPosture2D.cpp
    )

target_link_libraries(romea_path ${catkin_LIBRARIES} gsl blas)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS trackbod_driver trackbod_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif(CATKIN_ENABLE_TESTING)

