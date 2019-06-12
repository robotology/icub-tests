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

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>
#include <yarp/os/Time.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/sig/Vector.h>

#include "GazeControlSimpleLookTest.h"

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(GazeControlSimpleLookTest)


/***********************************************************************************/
GazeControlSimpleLookTest::GazeControlSimpleLookTest() :
                           yarp::robottestingframework::TestCase("GazeControlSimpleLookTest")
{
}


/***********************************************************************************/
GazeControlSimpleLookTest::~GazeControlSimpleLookTest()
{
}


/***********************************************************************************/
bool GazeControlSimpleLookTest::setup(Property &property)
{
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local",("/"+getName()+"/gaze"));

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Opening Gaze Controller Client");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver.open(option),"Unable to open the client!");
    return true;
}


/***********************************************************************************/
void GazeControlSimpleLookTest::tearDown()
{
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Closing Gaze Controller Client");
    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(driver.close(),"Unable to close the client!");
}


/***********************************************************************************/
void GazeControlSimpleLookTest::run()
{
    IGazeControl *igaze;
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(driver.view(igaze),"Opening the view on the device!");

    bool done;

    Vector fp;
    double t0=Time::now();
    while (Time::now()-t0<5.0)
    {
        done=igaze->getFixationPoint(fp);
        if (done)
            break;
        Time::delay(0.1);
    }
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Initial fixation-point retrieved!");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Setting up the context");
    int context;
    igaze->storeContext(&context);

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Looking at the target");
    Vector fpd(3,0.0); fp[0]=-0.4;
    igaze->lookAtFixationPoint(fpd);

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Waiting");
    igaze->waitMotionDone(1.0,5.0);

    igaze->checkMotionDone(&done);
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Target reached!");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Going back");
    igaze->lookAtFixationPoint(fp);

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Waiting");
    igaze->waitMotionDone(1.0,5.0);

    igaze->checkMotionDone(&done);
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Done!");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Cleaning up the context");
    igaze->restoreContext(context);
    igaze->deleteContext(context);
}

