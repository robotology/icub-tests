#!/bin/bash
#######################################################################################
# Copyright: (C) 2017
# Permission is granted to copy, distribute, and/or modify this program
# under the terms of the GNU General Public License, version 2 or any
# later version published by the Free Software Foundation.
#  *
# A copy of the license can be found at
# http://www.robotcub.org/icub/license/gpl.txt
#  *
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details
#######################################################################################


#######################################################################################
# USEFUL FUNCTIONS:                                                                  #
#######################################################################################
usage() {
cat << EOF
***************************************************************************************
IMU TEST
Author:  Martina Gloria   <martina.gloria@iit.it>

USAGE:
        $0 options

***************************************************************************************
OPTIONS:

***************************************************************************************
EXAMPLE USAGE:

***************************************************************************************
EOF
}

#######################################################################################
# FUNCTIONS:                                                                         #
#######################################################################################


go_home_helper() {

    go_home_helperT $1
    go_home_helperR $1
    go_home_helperL $1
    go_home_helperH $1
}

go_home_helperL()
{
    echo "ctpq time $1 off 0 pos (-6.0 23.0 25.0 29.0 -24.0 -3.0 -3.0)" | yarp rpc /ctpservice/left_arm/rpc
}


go_home_helperT()
{
    echo "ctpq time 1.0 off 0 pos (0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
}

go_home_helperR()
{
    echo "ctpq time $1 off 0 pos (-6.0 23.0 25.0 29.0 -24.0 -3.0 -3.0)" | yarp rpc /ctpservice/right_arm/rpc
}

go_home_helperH()
{
    echo "ctpq time $1 off 0 pos (0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
}

go_homeH() {
    head "stop"
    go_home_helperH 1.5
    sleep 2.0
    head "start"
}

go_home() {
    go_home_helper 5.0
    sleep 2.5
}


hello() {
 echo "ctpq time 1.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0 -11.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 2.0
    echo "ctpq time 0.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0  25.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 0.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0 -11.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 0.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0  25.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 0.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0 -11.0)" | yarp rpc /ctpservice/left_arm/rpc
}


look_hands() {
    echo "ctpq time 5.0 off 0 pos ( -3.0 57.0   3.0 106.0 -9.0 -8.0 -10.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 5.0 off 0 pos ( -3.0 57.0   3.0 106.0 -9.0 -8.0 -10.0)" | yarp rpc /ctpservice/left_arm/rpc
}

open_arms() {
    echo "ctpq time 5.0 off 0 pos (-27.0 78.0 -37.0 33.0 -79.0 0.0 -4.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 5.0 off 0 pos (-27.0 78.0 -37.0 33.0 -79.0 0.0 -4.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 2.0 off 0 pos (-27.0 78.0 -37.0 93.0 -79.0 0.0 -4.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 2.0 off 0 pos (-27.0 78.0 -37.0 93.0 -79.0 0.0 -4.0)" | yarp rpc /ctpservice/left_arm/rpc
}

look_down() {
    echo "ctpq time 5 off 0 pos (-29.8609 0.0659181 1.6095)" | yarp rpc /ctpservice/head/rpc
}

look_up() {
    echo "ctpq time 5 off 0 pos (1.07666 0.0659181 1.69739)" | yarp rpc /ctpservice/head/rpc
}

arms_up() {
    echo "ctpq time 5 off 0 pos (-90.0 60.0 20.0 45.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 5 off 0 pos (-90.0 60.0 20.0 45.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

#######################################################################################
# SEQUENCE:                                                                           #
#######################################################################################


sequence(){

  look_down
  look_up

  look_hands
  open_arms
  arms_up

  go_home
}


#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
echo "********************************************************************************"
echo ""

$1 "$2"

if [[ $# -eq 0 ]] ; then
    echo "No options were passed!"
    echo ""
    usage
    exit 1
fi