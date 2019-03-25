# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Nicolo' Genesio <nicolo.genesio@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 3.5)
endif()

# set the project name
project(SkinWrapperTest)

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

# add the source codes to build the plugin library
rtf_add_plugin(${PROJECT_NAME}  HEADERS ${PROJECT_NAME}.h
                                SOURCES ${PROJECT_NAME}.cpp)

# add required libraries
target_link_libraries(${PROJECT_NAME} ${RTF_LIBRARIES} ${YARP_LIBRARIES} YARP::YARP_rtf)

# set the installation options
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)

