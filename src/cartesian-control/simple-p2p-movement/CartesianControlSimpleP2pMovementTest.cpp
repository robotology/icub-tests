/* 
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <string>
#include <rtf/dll/Plugin.h>
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
                                       YarpTestCase("CartesianControlSimpleP2pMovementTest")
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
    RTF_ASSERT_FAIL_IF(driver.open(option),"Unable to open the client!");
    return true;
}


/***********************************************************************************/
void CartesianControlSimpleP2pMovementTest::tearDown()
{
    RTF_TEST_REPORT("Closing Cartesian Controller Client");
    RTF_ASSERT_FAIL_IF(driver.close(),"Unable to close the client!");
}


/***********************************************************************************/
void CartesianControlSimpleP2pMovementTest::run()
{
    ICartesianControl *iarm;
    RTF_ASSERT_FAIL_IF(driver.view(iarm),"Unable to correctly view the driver!");

    RTF_TEST_REPORT("Setting up the Controller");
    Vector dof;
    iarm->getDOF(dof);
    iarm->setDOF(dof,dof);
    iarm->setTrajTime(1.0);

    RTF_TEST_REPORT("Reaching for the target");
    Vector x(3,0.0); x[0]=-0.4;
    iarm->goToPositionSync(x);

    RTF_TEST_REPORT("Waiting");
    iarm->waitMotionDone(1.0,5.0);

    bool done;
    iarm->checkMotionDone(&done);
    RTF_TEST_FAIL_IF(done,"Unable to reach for the specified target!");
}

