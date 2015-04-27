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

# Build example?
add_subdirectory(example)

# Build camera tests
add_subdirectory(src/camera)

# Build position direct test
add_subdirectory(src/positionDirect)

# Build position direct test
add_subdirectory(src/opticalEncoders)

# Build model consistency check
add_subdirectory(src/models-consistency)

# Build motor tests
add_subdirectory(src/motor-tests)

# Build force sensor tests
add_subdirectory(src/ftsensor-tests)

# Build ports frequency tests
add_subdirectory(src/ports-frequency)
