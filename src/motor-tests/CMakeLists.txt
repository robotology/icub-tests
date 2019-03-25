#  Robot Testing Framework (RTF)
#  Copyright: (C) 2015 Ali Paikan
#  Authors: Ali Paikan <ali.paikan@gmail.com>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 3.5)
endif()

# set the project name
set(PROJECTNAME MotorTest)
project(${PROJECTNAME})

# add the source codes to build the plugin library
rtf_add_plugin(${PROJECT_NAME}    HEADERS MotorTest.h
                                  SOURCES MotorTest.cpp)

# add required libraries 
target_link_libraries(${PROJECTNAME} ${RTF_LIBRARIES}
                      ${YARP_LIBRARIES} YARP::YARP_rtf)


# set the installation options
install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)

