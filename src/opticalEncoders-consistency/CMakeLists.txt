#  RTF
#  Copyright: (C) 2015 Ali Paikan
#  Authors: Ali Paikan <ali.paikan@gmail.com>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#


cmake_minimum_required(VERSION 2.8.9)
set(PROJECTNAME opticalEncodersConsistency)
project(${PROJECTNAME})

find_package(RTF)
find_package(RTF COMPONENTS DLL)
find_package(YARP)

include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})

link_libraries(${RTF_LIBRARIES}
               ${YARP_LIBRARIES})

add_library(${PROJECTNAME} MODULE opticalEncodersConsistency.h
                                  opticalEncodersConsistency.cpp)
install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)
