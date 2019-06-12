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

#include <cstdio>
#include <ctime>
#include <robottestingframework/dll/Plugin.h>
#include "ExampleFixture.h"
#include <yarp/os/Property.h>
#include <yarp/os/Random.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;

ROBOTTESTINGFRAMEWORK_PREPARE_FIXTURE_PLUGIN(ExampleFixture)

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
