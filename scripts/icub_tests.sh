#!/bin/bash

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

size=`xrandr -q | awk -F'current' -F',' 'NR==1 {gsub("( |current)","");print $2}'`
w=`echo $size | cut -f1 -d'x'`
h=`echo $size | cut -f2 -d'x'`
#echo $((w / 2))

#rm -rf .G1 .G2 .G3 .G4
google-chrome --user-data-dir=.G1 --window-size=$((w / 2 - 5)),$((h / 2 - 5)) --window-position=0,0 --app="http://127.0.0.1:8080" &
google-chrome --user-data-dir=.G2 --window-size=$((w / 2 - 5)),$((h / 2 - 5)) --window-position=$((w / 2)),0 --app="http://127.0.0.1:8081" &
google-chrome --user-data-dir=.G3 --window-size=$((w / 2 - 5)),$((h / 2 - 5)) --window-position=0,$((h / 2)) --app="http://127.0.0.1:8082" &
google-chrome --user-data-dir=.G4 --window-size=$((w / 2 - 5)),$((h / 2 - 5)) --window-position=$((w / 2)),$((h / 2)) --app="http://127.0.0.1:8083" &

# running the tests 
dt=`date +%Y-%m-%d`
tm=`date +"%T"`
mkdir -p $dt
testrunner -w --web-port 8080 -o $dt/camera-icubSim_$tm.log   -s $ICUB_TESTS_ROOT/suits/camera-icubSim.xml
#testrunner -w --web-port 8081 -o $dt/basics-icubSim_$tm.log   -s $ICUB_TESTS_ROOT/suits/basics-icubSim.xml
#testrunner -w --web-port 8082 -o $dt/motors-icubSim_$tm.log   -s $ICUB_TESTS_ROOT/suits/motors-icubSim.xml
#testrunner -w --web-port 8083 -o $dt/encoders-icubSim_$tm.log -s $ICUB_TESTS_ROOT/suits/encoders-icubSim.xml

