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
    )

find_package(Eigen REQUIRED)
find_package(sophus REQUIRED)

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
    DEPENDS
        Eigen
        sophus
    )

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
add_definitions(${catkin_DEFINITIONS} ${sophus_DEFINITIONS} ${Eigen_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS} ${sophus_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

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

