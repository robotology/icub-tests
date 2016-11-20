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

#include <rtf/dll/Plugin.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/sig/Vector.h>

#include "GazeControlSimpleLookTest.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

// prepare the plugin
PREPARE_PLUGIN(GazeControlSimpleLookTest)


/***********************************************************************************/
GazeControlSimpleLookTest::GazeControlSimpleLookTest() :
                           YarpTestCase("GazeControlSimpleLookTest")
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

    RTF_TEST_REPORT("Opening Gaze Controller Client");
    RTF_ASSERT_ERROR_IF(driver.open(option),"Unable to open the client!");
    return true;
}


/***********************************************************************************/
void GazeControlSimpleLookTest::tearDown()
{
    RTF_TEST_REPORT("Closing Gaze Controller Client");
    RTF_ASSERT_FAIL_IF(driver.close(),"Unable to close the client!");
}


/***********************************************************************************/
void GazeControlSimpleLookTest::run()
{
    IGazeControl *igaze;
    RTF_TEST_CHECK(driver.view(igaze),"Opening the view on the device!");

    Vector fp;
    igaze->getFixationPoint(fp);

    RTF_TEST_REPORT("Setting up the context");
    int context;
    igaze->storeContext(&context);

    RTF_TEST_REPORT("Looking at the target");
    Vector fpd(3,0.0); fp[0]=-0.4;
    igaze->lookAtFixationPoint(fpd);

    RTF_TEST_REPORT("Waiting");
    igaze->waitMotionDone(1.0,5.0);

    bool done;
    igaze->checkMotionDone(&done);
    RTF_TEST_CHECK(done,"Target reached!");

    RTF_TEST_REPORT("Going back");
    igaze->lookAtFixationPoint(fp);

    RTF_TEST_REPORT("Waiting");
    igaze->waitMotionDone(1.0,5.0);

    igaze->checkMotionDone(&done);
    RTF_TEST_CHECK(done,"Done!");

    RTF_TEST_REPORT("Cleaning up the context");
    igaze->restoreContext(context);
    igaze->deleteContext(context);
}

