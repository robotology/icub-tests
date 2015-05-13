#  iCub Robot test units (Robot Testing Framework)
#  Copyright: (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Ali Paikan <ali.paikan@gmail.com>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#


cmake_minimum_required(VERSION 2.8.9)
project(iCub-Tests)

# find a version of RTF higher then 1.0.1
# and with yarp support enabled
find_package(RTF 1.0.1 COMPONENTS YARP REQUIRED)

# setting default compilation to release/optmized
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, recommanded options are: Debug or Release" FORCE)
endif()

# set the output plugin directory to collect all the shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# Build example?
add_subdirectory(example)

# Build camera tests
add_subdirectory(src/camera)

# Build position direct tests
add_subdirectory(src/positionDirect)

# Build openloop tests
add_subdirectory(src/openloop-consistency)

# Build torque control tests
add_subdirectory(src/torqueControl-consistency)

# Build optical encoder tests
add_subdirectory(src/opticalEncoders-consistency)
add_subdirectory(src/opticalEncoders-drift)

# Build model consistency check
add_subdirectory(src/models-consistency)

# Build motor tests
add_subdirectory(src/motor-tests)

# Build force sensor tests
add_subdirectory(src/ftsensor-tests)

# Build controlModes tests
add_subdirectory(src/controlModes)

# Build ports frequency tests
add_subdirectory(src/ports-frequency)
