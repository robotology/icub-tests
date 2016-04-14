#  RTF
#  Copyright: (C) 2016 Nuno Guedelha
#  Authors: Nuno Guedelha <nuno.guedelha@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#


cmake_minimum_required(VERSION 2.8.9)

# set the project name
set(PROJECTNAME AccelerometersReading)
project(${PROJECTNAME})

# add the required cmake packages
find_package(RTF)
find_package(RTF COMPONENTS DLL)
find_package(YARP)

# add include directories
include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})

# add required libraries
link_libraries(${RTF_LIBRARIES}
               ${YARP_LIBRARIES})

# add the source codes to build the plugin library
add_library(${PROJECTNAME} MODULE IMTBsensorParser.h
                                  MTBsensorParserCan.h
                                  MTBsensorParserEth.h
                                  AccelerometersReading.h
                                  InterfaceDestructors.cpp
                                  MTBsensorParserCan.cpp
                                  MTBsensorParserEth.cpp
                                  AccelerometersReading.cpp)

# set the installation options
install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)
