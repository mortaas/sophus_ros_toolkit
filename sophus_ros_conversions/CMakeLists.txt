##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(sophus_ros_conversions)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin
    REQUIRED
    COMPONENTS
        cmake_modules
        ecl_build
        geometry_msgs
        tf
    )

find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# Sophus - C++ implementation of Lie Groups using Eigen
## Set Sophus headers path
## ENV reads paths from a system environment variable
set(SOPHUS "$ENV{HOME}/lib/sophus")
set(sophus_INCLUDE_DIRS ${SOPHUS})
find_package(Sophus 1.0.0 EXACT REQUIRED PATHS ${SOPHUS}/build NO_DEFAULT_PATH)

##############################################################################
# Catkin
##############################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        cmake_modules
        ecl_build
        geometry_msgs
        tf
    DEPENDS
        EIGEN3
    )

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
add_definitions(${catkin_DEFINITIONS} ${sophus_DEFINITIONS} ${EIGEN3_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS} ${sophus_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(src)

##############################################################################
# Installs
##############################################################################

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

#install(DIRECTORY doc DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

