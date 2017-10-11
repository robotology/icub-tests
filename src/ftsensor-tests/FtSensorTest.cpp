// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <cstdlib>
#include <rtf/dll/Plugin.h>
#include <rtf/TestAssert.h>

#include <yarp/os/Time.h>

#include "FtSensorTest.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;

// prepare the plugin
PREPARE_PLUGIN(FtSensorTest)

FtSensorTest::FtSensorTest() : yarp::rtf::TestCase("FtSensorTest") {
}

FtSensorTest::~FtSensorTest() { }

bool FtSensorTest::setup(yarp::os::Property &configuration) {
    // initialization goes here ...
    if(configuration.check("name"))
        setName(configuration.find("name").asString());

    RTF_ASSERT_ERROR_IF_FALSE(configuration.check("portname"),
                        "Missing 'portname' parameter");
    portname = configuration.find("portname").asString();

    RTF_ASSERT_ERROR_IF_FALSE(port.open("/iCubTest/FTsensor"),
                        "opening port, is YARP network working?");

    RTF_TEST_REPORT(Asserter::format("connecting from %s to %s\n",
                                     port.getName().c_str(), portname.c_str()));

    RTF_ASSERT_ERROR_IF_FALSE(Network::connect(portname, port.getName()),
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
    RTF_TEST_REPORT("Reading FT sensors...");
    Vector *readSensor = port.read();
    RTF_TEST_FAIL_IF(readSensor, "could not read FT data from sensor");

    RTF_TEST_FAIL_IF(readSensor->size() == 6, "sensor has 6 values");
}

