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


cmake_minimum_required(VERSION 3.5)

project(iCub-Tests)

find_package(RobotTestingFramework 1.6.51 COMPONENTS DLL REQUIRED)
find_package(YARP 3.1.101 COMPONENTS math robottestingframework REQUIRED)

# set the output plugin directory to collect all the shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# options
option(ICUB_TESTS_USES_ICUB_MAIN "Turn on to compile the tests that depend on the icub-main repository" OFF)
option(ICUB_TESTS_USES_CODYCO    "Turn on to compile the test that depend on the codyco-superbuil repository" OFF)

# Build examples?
add_subdirectory(example/cpp)

# Build camera tests
add_subdirectory(src/camera)

# Build position direct tests
add_subdirectory(src/positionDirect)

# Build positionControl-accuracy tests
add_subdirectory(src/positionControl-accuracy)
add_subdirectory(src/positionControl-accuracy-ExternalPid)

# Build openloop tests
add_subdirectory(src/openloop-consistency)

# Build torque control tests
add_subdirectory(src/torqueControl-consistency)
add_subdirectory(src/torqueControl-stiffDampCheck)

# Build cartesian controller tests
add_subdirectory(src/cartesian-control)

# Build gaze controller tests
add_subdirectory(src/gaze-control)

# Build demoRedBall test
add_subdirectory(src/demoRedBall)

if(ICUB_TESTS_USES_CODYCO)
    add_subdirectory(src/torqueControl-gravityConsistency)
endif()

# Build optical encoder tests
add_subdirectory(src/motorEncoders-consistency)
add_subdirectory(src/opticalEncoders-drift)
add_subdirectory(src/motorEncodersSignCheck)

# Build model consistency check
if(ICUB_TESTS_USES_ICUB_MAIN)
    add_subdirectory(src/models-consistency)
endif()

# Build motor tests
add_subdirectory(src/motor-tests)
add_subdirectory(src/jointLimits)
add_subdirectory(src/motor-stiction)

# Build force sensor tests
add_subdirectory(src/ftsensor-tests)

# Build controlModes tests
add_subdirectory(src/controlModes)

# Build ports frequency tests
add_subdirectory(src/ports-frequency)

#interfeces
add_subdirectory(src/movementReferencesTest)

# Build skinWrapper tests
add_subdirectory(src/skinWrapperTest)

# Build system status tests (needs yarp newer than f0c9e83a66d1e2685361f82dd98b20ed6479ec04)
#add_subdirectory(src/system-status)


