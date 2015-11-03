// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <rtf/dll/Plugin.h>
#include "ExampleTest.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;

// prepare the plugin
PREPARE_PLUGIN(ExampleTest)

ExampleTest::ExampleTest() : YarpTestCase("ExampleTest") {
}

ExampleTest::~ExampleTest() { }

bool ExampleTest::setup(yarp::os::Property &property) {

    // initialization goes here ...
    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    string example = property.check("example", Value("default value")).asString();

    RTF_TEST_REPORT(Asserter::format("Use '%s' for the example param!",
                                       example.c_str()));
    return true;
}

void ExampleTest::tearDown() {
    // finalization goes her ...
}

void ExampleTest::run() {

    int a = 5; int b = 3;
    RTF_TEST_REPORT("testing a < b");
    RTF_TEST_FAIL_IF(a<b, Asserter::format("%d is not smaller than %d.", a, b));
    RTF_TEST_REPORT("testing a > b");
    RTF_TEST_FAIL_IF(a>b, Asserter::format("%d is not smaller than %d.", a, b));
    RTF_TEST_REPORT("testing a == b");
    RTF_TEST_FAIL_IF(a==b, Asserter::format("%d is not smaller than %d.", a, b));
    // add more
    // ...
}

