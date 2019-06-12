/*
 * iCub Robot Unit Tests (Robot Testing Framework)
 *
 * Copyright (C) 2015-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <cstdlib>
#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

#include <yarp/os/Time.h>

#include "FtSensorTest.h"

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::sig;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(FtSensorTest)

FtSensorTest::FtSensorTest() : yarp::robottestingframework::TestCase("FtSensorTest") {
}

FtSensorTest::~FtSensorTest() { }

bool FtSensorTest::setup(yarp::os::Property &configuration) {
    // initialization goes here ...
    if(configuration.check("name"))
        setName(configuration.find("name").asString());

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(configuration.check("portname"),
                        "Missing 'portname' parameter");
    portname = configuration.find("portname").asString();

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(port.open("/iCubTest/FTsensor"),
                        "opening port, is YARP network working?");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("connecting from %s to %s\n",
                                     port.getName().c_str(), portname.c_str()));

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(Network::connect(portname, port.getName()),
                        Asserter::format("could not connect to remote port %s, FT sensor unavailable",
                                         portname.c_str()));
    return true;
}

void FtSensorTest::tearDown() {
    // finalization goes here ...
    Network::disconnect(portname, port.getName());
    port.close();
}

void FtSensorTest::run() {
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Reading FT sensors...");
    Vector *readSensor = port.read();
    ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(readSensor, "could not read FT data from sensor");

    ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(readSensor->size() == 6, "sensor has 6 values");
}

