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

#include "ExampleTest.h"
#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(ExampleTest)

ExampleTest::ExampleTest() : yarp::robottestingframework::TestCase("ExampleTest") {
}

ExampleTest::~ExampleTest() { }

bool ExampleTest::setup(yarp::os::Property &property) {

    // initialization goes here ...
    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    string example = property.check("example", Value("default value")).asString();

    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Use '%s' for the example param!",
                                       example.c_str()));
    return true;
}

void ExampleTest::tearDown() {
    // finalization goes her ...
}

void ExampleTest::run() {

    int a = 5; int b = 3;
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(a<b, "a smaller then b");
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(a>b, "a bigger then b");
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(a==b, "a equal to b");

    // add more
    // ...
}

