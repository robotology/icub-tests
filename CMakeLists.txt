#  iCub Robot test units (Robot Testing Framework)
#  Copyright: (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Ali Paikan <ali.paikan@gmail.com>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#


cmake_minimum_required(VERSION 2.8.9)
project(Robot-Tests)

# set the include dir for helper classes
set(YARP_HELPERS_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

# set the output plugin directory to collect all the shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# Build yarp fixture manager plugin
# add_subdirectory(src/fixture-ymanager)

# Build example?
add_subdirectory(example)

# Build camera tests
add_subdirectory(src/camera)

# Build model consistency check
add_subdirectory(src/models-consistency)

