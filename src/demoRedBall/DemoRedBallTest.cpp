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
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include "DemoRedBallTest.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

// prepare the plugin
PREPARE_PLUGIN(DemoRedBallTest)


/***********************************************************************************/ 
class DemoRedBallPosition : public RateThread
{
    string name;    
    IGazeControl *igaze;
    string eye;
    Vector pos;
    BufferedPort<Bottle> port;

    bool threadInit()
    {
        port.open(("/"+name+"/redballpos:o"));
    }

    void run()
    {
        if (igaze!=NULL)
        {
            Vector x,o;
            if (eye=="left")
                gazeCtrl->getLeftEyePose(x,o);
            else
                gazeCtrl->getRightEyePose(x,o);

            Matrix T=axis2dcm(o);
            T.setSubcol(x,0,3);
            Vector pos_=T*pos;

            Bottle &cmd=port.prepare();
            cmd.clear();
            cmd.addDouble(pos_[0]);
            cmd.addDouble(pos_[1]);
            cmd.addDouble(pos_[2]);
            cmd.addDouble(0.0);
            cmd.addDouble(0.0);
            cmd.addDouble(0.0);
            cmd.addDouble(1.0);
            port.write();
        }
    }

    void threadRelease()
    {
        port.close();
    }

public:
    DemoRedBallPosition(const string &name_,
                        IGazeControl *igaze_,
                        const string &eye_) :
                        RateThread(100), name(name_),
                        igaze(igaze_), eye(eye_),
                        pos(4,0.0)
    {
        pos[3]=1.0;
    }

    bool setPos(const Vector &pos)
    {
        if (pos.length()>=3)
        {
            this->pos.setSubvector(0,pos.subVector(0,2));
            return true;
        }
        else
            return false;
    }
};


/***********************************************************************************/
DemoRedBallTest::DemoRedBallTest() : YarpTestCase("DemoRedBallTest"), redBallPos(NULL)
{
}


/***********************************************************************************/
DemoRedBallTest::~DemoRedBallTest()
{
    delete redBallPos;
}


/***********************************************************************************/
bool DemoRedBallTest::setup(Property &property)
{
    string context=property.check("context",Value("demoRedBall")).asString();
    string from=property.check("from",Value("config.ini")).asString();

    Property option;
    option.put("device","cartesiancontrollerclient");
    option.put("remote",("/"+robot+"/"+"cartesianController/"+arm+"_arm"));
    option.put("local",("/"+getName()+"/"+arm+"_arm"));

    RTF_TEST_REPORT(Asserter::format("Opening Cartesian Controller Client for %s_arm",arm.c_str()));
    RTF_ASSERT_ERROR_IF(driver.open(option),"Unable to open the client!");
    return true;
}


/***********************************************************************************/
void DemoRedBallTest::tearDown()
{
    RTF_TEST_REPORT("Closing Cartesian Controller Client");
    RTF_ASSERT_FAIL_IF(driver.close(),"Unable to close the client!");
}


/***********************************************************************************/
void DemoRedBallTest::run()
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
    iarm->waitMotionDone(1.0,5.0);

    iarm->checkMotionDone(&done);
    RTF_TEST_CHECK(done,"Target reached!");

    RTF_TEST_REPORT("Going back to starting pose");
    iarm->setLimits(0,0.0,0.0);
    iarm->setLimits(1,0.0,0.0);
    iarm->setLimits(2,0.0,0.0);
    iarm->goToPoseSync(x,o);

    RTF_TEST_REPORT("Waiting");
    iarm->waitMotionDone(1.0,5.0);

    iarm->checkMotionDone(&done);
    RTF_TEST_CHECK(done,"Starting pose reached!");

    RTF_TEST_REPORT("Cleaning up the context");
    iarm->restoreContext(context);
    iarm->deleteContext(context);
}

