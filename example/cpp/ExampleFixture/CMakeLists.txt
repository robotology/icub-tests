# iCub Robot Unit Tests (Robot Testing Framework)
#
# Copyright (C) 2015-2019 Istituto Italiano di Tecnologia (IIT)
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA


if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 3.5)
endif()

project(ExampleFixture)

# add the required cmake packages
find_package(RobotTestingFramework 2 COMPONENTS DLL)
find_package(YARP 3.3.0 COMPONENTS os robottestingframework)

# add the source codes to build the plugin library
add_library(${PROJECT_NAME} MODULE ExampleFixture.h
                                   ExampleFixture.cpp)

# add required libraries
target_link_libraries(${PROJECT_NAME} RobotTestingFramework::RTF
                                      RobotTestingFramework::RTF_dll
                                      YARP::YARP_os
                                      YARP::YARP_init
                                      YARP::YARP_robottestingframework)

# set the installation options
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)
