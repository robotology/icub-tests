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

#include <string>
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>

#include "CartesianControlSimpleP2pMovementTest.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

// prepare the plugin
PREPARE_PLUGIN(CartesianControlSimpleP2pMovementTest)


/***********************************************************************************/
CartesianControlSimpleP2pMovementTest::CartesianControlSimpleP2pMovementTest() :
                                       yarp::rtf::TestCase("CartesianControlSimpleP2pMovementTest")
{
}


/***********************************************************************************/
CartesianControlSimpleP2pMovementTest::~CartesianControlSimpleP2pMovementTest()
{
}


/***********************************************************************************/
bool CartesianControlSimpleP2pMovementTest::setup(Property &property)
{
    string robot=property.check("robot",Value("icubSim")).asString();
    string arm=property.check("arm-type",Value("left")).asString();

    Property option;
    option.put("device","cartesiancontrollerclient");
    option.put("remote",("/"+robot+"/"+"cartesianController/"+arm+"_arm"));
    option.put("local",("/"+getName()+"/"+arm+"_arm"));

    RTF_TEST_REPORT(Asserter::format("Opening Cartesian Controller Client for %s_arm",arm.c_str()));
    RTF_ASSERT_ERROR_IF_FALSE(driver.open(option),"Unable to open the client!");
    return true;
}


/***********************************************************************************/
void CartesianControlSimpleP2pMovementTest::tearDown()
{
    RTF_TEST_REPORT("Closing Cartesian Controller Client");
    RTF_ASSERT_FAIL_IF_FALSE(driver.close(),"Unable to close the client!");
}


/***********************************************************************************/
void CartesianControlSimpleP2pMovementTest::run()
{
    ICartesianControl *iarm;
    RTF_TEST_CHECK(driver.view(iarm),"Opening the view on the device!");

    bool done;

    Vector x,o;    
    double t0=Time::now();
    while (Time::now()-t0<5.0)
    {
        done=iarm->getPose(x,o);
        if (done)
            break;
        Time::delay(0.1);
    }
    RTF_TEST_CHECK(done,"Initial pose retrieved!");

    RTF_TEST_REPORT("Setting up the context");
    int context;
    iarm->storeContext(&context);

    Vector dof;
    iarm->getDOF(dof); dof=1.0;
    iarm->setDOF(dof,dof);
    iarm->setTrajTime(1.0);

    RTF_TEST_REPORT("Reaching for the target");
    Vector xd(3,0.0); xd[0]=-0.4;
    iarm->goToPositionSync(xd);

    RTF_TEST_REPORT("Waiting");
    done=iarm->waitMotionDone(1.0,5.0);
    RTF_TEST_CHECK(done,"Target reached!");

    RTF_TEST_REPORT("Going back to starting pose");
    iarm->setLimits(0,0.0,0.0);
    iarm->setLimits(1,0.0,0.0);
    iarm->setLimits(2,0.0,0.0);
    iarm->goToPoseSync(x,o);

    RTF_TEST_REPORT("Waiting");
    done=iarm->waitMotionDone(1.0,5.0);
    RTF_TEST_CHECK(done,"Starting pose reached!");

    RTF_TEST_REPORT("Cleaning up the context");
    iarm->restoreContext(context);
    iarm->deleteContext(context);
}

