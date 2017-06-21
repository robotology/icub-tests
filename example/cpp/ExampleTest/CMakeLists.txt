#  Robot Testing Framework (RTF)
#  Copyright: (C) 2015 Ali Paikan
#  Authors: Ali Paikan <ali.paikan@gmail.com>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

cmake_minimum_required(VERSION 3.0)

# set the project name
set(PROJECTNAME ExampleTest)
project(${PROJECTNAME})

# add the required cmake packages
find_package(RTF 1.1 COMPONENTS DLL)
find_package(YARP)

# add include directories
include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

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

