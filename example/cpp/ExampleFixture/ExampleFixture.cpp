// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <cstdio>
#include <ctime>
#include <rtf/dll/Plugin.h>
#include "ExampleFixture.h"
#include <yarp/os/Property.h>
#include <yarp/os/Random.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;

PREPARE_FIXTURE_PLUGIN(ExampleFixture)

bool ExampleFixture::setup(int argc, char** argv) {
    printf("ExampleFixture: setupping fixture...\n");
    // do the setup here
    Property prop;
    prop.fromCommand(argc, argv, false);
    if(!prop.check("probability")) {
        printf("ExampleFixture: missing 'probability' param.\n");
        return false;
    }
    probability = prop.find("probability").asDouble();
    Random::seed(time(NULL));
    return true;
}

bool ExampleFixture::check() {
    // return a random boolean
    return ((double)Random::uniform(0, 100) <= 100.0*probability);
}

void ExampleFixture::tearDown() {
    printf("ExampleFixture: tearing down the fixture...\n");
}
