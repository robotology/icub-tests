#  RTF
#  Copyright: (C) 2015 Valentina Gaggero
#  Authors: Valentina Gaggero <valentina.gaggero@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 3.5)
endif()

set(PROJECTNAME TorqueControlStiffDampCheck)
project(${PROJECTNAME})

rtf_add_plugin(${PROJECT_NAME}    HEADERS TorqueControlStiffDampCheck.h
                                  SOURCES TorqueControlStiffDampCheck.cpp)

target_link_libraries(${PROJECTNAME} ${RTF_LIBRARIES}
                      ${YARP_LIBRARIES} YARP::YARP_rtf)

install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)
