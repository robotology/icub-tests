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

# set the output plugin directory to collect all the shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# options
option(ICUB_TESTS_USES_ICUB_MAIN "Turn on to compile the tests that depend on the icub-main repository" OFF)

# Build example?
add_subdirectory(example/cpp)

# Build camera tests
add_subdirectory(src/camera)

# Build position direct tests
add_subdirectory(src/positionDirect)

# Build openloop tests
add_subdirectory(src/openloop-consistency)

# Build torque control tests
add_subdirectory(src/torqueControl-consistency)

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

# Build system status tests (needs yarp newer than f0c9e83a66d1e2685361f82dd98b20ed6479ec04)
#add_subdirectory(src/system-status)


