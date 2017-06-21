#  RTF
#  Copyright: (C) 2015 Valentina Gaggero
#  Authors: Valentina Gaggero <valentina.gaggero@iit.it >
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#


cmake_minimum_required(VERSION 3.0)
set(PROJECTNAME movementReferencesTest)
project(${PROJECTNAME})

find_package(RTF 1.1 COMPONENTS DLL)
find_package(YARP)

include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    )

add_library(${PROJECTNAME} MODULE movementReferencesTest.h
                                  movementReferencesTest.cpp
                                  )

target_link_libraries(${PROJECTNAME} ${RTF_LIBRARIES}
                      ${YARP_LIBRARIES} YARP::YARP_rtf
                      YARP::YARP_manager)

install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)
