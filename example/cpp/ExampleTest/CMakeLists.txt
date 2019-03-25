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
set(PROJECTNAME ExampleTest)
project(${PROJECTNAME})

# add the required cmake packages
find_package(RTF 1.1 COMPONENTS DLL)
find_package(YARP)

# add the source codes to build the plugin library
add_library(${PROJECTNAME} MODULE ExampleTest.h
                                  ExampleTest.cpp)
# add required libraries 
target_link_libraries(${PROJECTNAME} ${RTF_LIBRARIES}
                                     ${YARP_LIBRARIES}
                                     YARP::YARP_rtf)

# set the installation options
install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)

