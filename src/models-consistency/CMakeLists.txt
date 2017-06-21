#  Robot Testing Framework (RTF)
#  Copyright: (C) 2015 Ali Paikan
#  Authors: Ali Paikan <ali.paikan@gmail.com>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

cmake_minimum_required(VERSION 3.0)

# set the project name
set(PROJECTNAME iKiniDynConsistencyTest)
project(${PROJECTNAME})

# add the required cmake packages
find_package(RTF 1.1 COMPONENTS DLL)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

INCLUDE(iCubHelpers)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

# add include directories
include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

# add the source codes to build the plugin library
add_library(${PROJECTNAME} MODULE iKiniDynConsistencyTest.h
                                  iKiniDynConsistencyTest.cpp)
# add required libraries
target_link_libraries(${PROJECTNAME} ${RTF_LIBRARIES}
               ${YARP_LIBRARIES} YARP::YARP_rtf
               iKin iDyn)

# set the installation options
install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)

