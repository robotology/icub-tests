// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan and Lorenzo Natale
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#include <iostream>
#include <stdlib.h>     // for abs()
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include "CameraTest.h"

using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;

#define TIMES       1
#define FREQUENCY   30
#define TOLERANCE   5


// prepare the plugin
PREPARE_PLUGIN(CameraTest)

CameraTest::CameraTest() : YarpTestCase("CameraTest") {
}

CameraTest::~CameraTest() { }

bool CameraTest::setup(yarp::os::Property& property) {

    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("portname"),
                        "The portname must be given as the test paramter!");
    cameraPortName = property.find("portname").asString();
    measure_time = property.check("measure_time") ? property.find("measure_time").asInt() : TIMES;
    expected_frequency = property.check("expected_frequency") ? property.find("expected_frequency").asInt() : FREQUENCY;
    tolerance = property.check("tolerance") ? property.find("tolerance").asInt() : TOLERANCE;

    // opening port
    RTF_ASSERT_ERROR_IF(port.open("/CameraTest/image:i"),
                        "opening port, is YARP network available?");

    RTF_TEST_REPORT(Asserter::format("Listening to camera for %d seconds",
                                       measure_time));

    // connecting
    RTF_TEST_REPORT(Asserter::format("connecting from %s to %s",
                                       port.getName().c_str(), cameraPortName.c_str()));
    RTF_ASSERT_ERROR_IF(Network::connect(cameraPortName, port.getName()),
                     "could not connect to remote port, camera unavailable");
    return true;
}

void CameraTest::tearDown() {
    Network::disconnect(cameraPortName, port.getName());
    port.close();
}

void CameraTest::run() {
    RTF_TEST_REPORT("Reading images...");
    double timeStart=yarp::os::Time::now();
    double timeNow=timeStart;

    int frames=0;
    while(timeNow<timeStart+measure_time) {
        Image *image=port.read(false);
        if(image!=0)
            frames++;
        yarp::os::Time::delay(0.01);
        timeNow=yarp::os::Time::now();
    }

    int expectedFrames = measure_time*expected_frequency;
    RTF_TEST_REPORT(Asserter::format("Received %d frames, expecting %d",
                                       frames,
                                       expectedFrames));
    RTF_TEST_FAIL_IF(abs(frames-expectedFrames)<tolerance,
                     "checking number of received frames");
}
