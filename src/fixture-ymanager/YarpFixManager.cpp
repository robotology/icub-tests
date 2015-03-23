// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <stdio.h>
#include <Plugin.h>
#include <TestAssert.h>

#include <yarp/manager/utility.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include "YarpFixManager.h"

using namespace RTF;
using namespace yarp::os;
using namespace yarp::manager;

PREPARE_FIXTURE_PLUGIN(YarpFixManager)

bool YarpFixManager::setup(int argc, char** argv) {

    // check yarp network
    yarp.setVerbosity(-1);
    RTF_ASSERT_ERROR_IF(yarp.checkNetwork(),
                        "YARP network does not seem to be available, is the yarp server accessible?");

    // load the config file and update the environment if available
    // E.g., "--from mytest.ini"
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("RobotTesting");
    rf.configure(argc, argv);

    printf("rf: %s\n", rf.toString().c_str());
    RTF_ASSERT_ERROR_IF(rf.check("application"),
                        "No application xml file is set (add --aplication yourfixture.xml)");

    //yarp::os::Property config;
    //config.fromString(rf.toString());


    printf("Called from fixture plugin: setupping fixture...\n");
    // do the setup here
    // ...
    return true;
}

void YarpFixManager::tearDown() {
    printf("Called from fixture plugin: tearing down the fixture...\n");
    // do the tear down here
    // ...
}

void YarpFixManager::onExecutableStart(void* which) {

}

void YarpFixManager::onExecutableStop(void* which) {

}

void YarpFixManager::onExecutableDied(void* which) {

}

void YarpFixManager::onExecutableFailed(void* which) {

}

void YarpFixManager::onCnnStablished(void* which) {

}

void YarpFixManager::onCnnFailed(void* which) {

}
