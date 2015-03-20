#  iCub Robot test units (Robot Testing Framework)
#  Copyright: (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Ali Paikan <ali.paikan@gmail.com>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#


cmake_minimum_required(VERSION 2.8.9)
project(Robot-Tests)

set(YARP_HELPERS_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

# Build example? 
add_subdirectory(example)

# Build camera tests 
add_subdirectory(src/camera)


