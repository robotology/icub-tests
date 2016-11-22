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

#include <algorithm>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/GazeControl.h>
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
        return port.open(("/"+name+"/redballpos:o"));
    }

    void run()
    {
        if (igaze!=NULL)
        {
            Vector x,o;
            if (eye=="left")
                igaze->getLeftEyePose(x,o);
            else
                igaze->getRightEyePose(x,o);

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
                        PolyDriver &driver,
                        const string &eye_) :
                        RateThread(100), name(name_),
                        eye(eye_), pos(4,0.0)
    {
        if (!driver.view(igaze))
            igaze=NULL;
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

    // retrieve demoRedBall parameters
    ResourceFinder rf; rf.setVerbose();
    rf.setDefaultContext(context.c_str());
    rf.setDefaultConfigFile(from.c_str());
    rf.configure(0,NULL);

    // fallback values
    params.robot="icubSim";
    params.eye="left";
    params.reach_tol=0.01;
    params.use_left=true;
    params.use_right=true;
    params.home_arm.resize(7,0.0);

    Bottle &general=rf.findGroup("general");
    if (!general.isNull())
    {
        params.robot=general.check("robot",Value(params.robot)).asString();
        params.eye=general.check("eye",Value(params.eye)).asString();
        params.reach_tol=general.check("reach_tol",Value(params.reach_tol)).asDouble();
        params.use_left=(general.check("left_arm",Value(params.use_left?"on":"off")).asString()=="on");
        params.use_right=(general.check("right_arm",Value(params.use_right?"on":"off")).asString()=="on");
    }

    Bottle &home_arm=rf.findGroup("home_arm");
    if (!home_arm.isNull())
    {
        if (home_arm.check("poss"))
        {
            Bottle &poss=home_arm.findGroup("poss");
            for (size_t i=0; i<std::min(params.home_arm.length(),(size_t)poss.size()-1); i++)
                params.home_arm[i]=poss.get(1+i).asDouble();
        }
    }
    
    RTF_TEST_REPORT("Opening Clients");
    if (params.use_left)
    {
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote",("/"+params.robot+"/"+"left_arm"));
        optJoint.put("local",("/"+getName()+"/joint/left_arm"));

        Property optCart;
        optCart.put("device","cartesiancontrollerclient");
        optCart.put("remote",("/"+params.robot+"/"+"cartesianController/left_arm"));
        optCart.put("local",("/"+getName()+"/cartesian/left_arm"));

        RTF_ASSERT_ERROR_IF(drvJointArmL.open(optJoint)&&drvCartArmL.open(optCart),
                            "Unable to open clients for left_arm!");
    }

    if (params.use_right)
    {
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote",("/"+params.robot+"/"+"right_arm"));
        optJoint.put("local",("/"+getName()+"/joint/right_arm"));

        Property optCart;
        optCart.put("device","cartesiancontrollerclient");
        optCart.put("remote",("/"+params.robot+"/"+"cartesianController/right_arm"));
        optCart.put("local",("/"+getName()+"/cartesian/right_arm"));

        RTF_ASSERT_ERROR_IF(drvJointArmR.open(optJoint)&&drvCartArmR.open(optCart),
                            "Unable to open clients for right_arm!");
    }

    {
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote",("/"+params.robot+"/"+"head"));
        optJoint.put("local",("/"+getName()+"/joint/head"));

        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local",("/"+getName()+"/gaze"));

        RTF_ASSERT_ERROR_IF(drvJointHead.open(optJoint)&&drvGaze.open(optGaze),
                            "Unable to open clients for head!");
    }

    {
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote",("/"+params.robot+"/"+"torso"));
        optJoint.put("local",("/"+getName()+"/joint/torso"));

        RTF_ASSERT_ERROR_IF(drvJointTorso.open(optJoint),
                            "Unable to open clients for torso!");
    }

    redBallPos=new DemoRedBallPosition(getName(),drvGaze,params.eye);
    return true;
}


/***********************************************************************************/
void DemoRedBallTest::tearDown()
{
    redBallPos->stop();

    RTF_TEST_REPORT("Closing Clients");
    RTF_ASSERT_FAIL_IF(drvJointArmL.close()&&drvCartArmL.close(),
                       "Unable to close client for left_arm!");
    RTF_ASSERT_FAIL_IF(drvJointArmR.close()&&drvCartArmR.close(),
                       "Unable to close client for right_arm!");
    RTF_ASSERT_FAIL_IF(drvJointHead.close()&&drvGaze.close(),
                       "Unable to close client for head!");
    RTF_ASSERT_FAIL_IF(drvJointTorso.close(),"Unable to close client for left_arm!");
}


/***********************************************************************************/
void DemoRedBallTest::testBallPosition(const Vector &pos)
{
    DemoRedBallPosition *ball=dynamic_cast<DemoRedBallPosition*>(redBallPos);
    ball->setPos(pos);

    if (!ball->isRunning())
        ball->start();
    else if (ball->isSuspended())
        ball->resume();
    
    Vector x,o,encs;
    int nEncs; IEncoders* ienc;
    bool done=false;
    double t0;
        
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        arm_under_test.iarm->getPose(x,o);
        if (norm(pos-x)<params.reach_tol)
        {
            done=true;
            break;
        }
        Time::delay(0.1);
    }
    RTF_TEST_CHECK(done,"Ball reached with the hand!");

    IGazeControl* igaze;
    drvGaze.view(igaze);
    t0=Time::now();
    while (Time::now()-t0<5.0)
    {
        igaze->getFixationPoint(x);
        if (norm(pos-x)<params.reach_tol)
        {
            done=true;
            break;
        }
        Time::delay(0.1);
    }
    RTF_TEST_CHECK(done,"Ball gazed at with the eyes!");

    RTF_TEST_REPORT("Going home");
    ball->suspend();
    
    arm_under_test.ienc->getAxes(&nEncs);
    encs.resize(nEncs,0.0);
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        arm_under_test.ienc->getEncoders(encs.data());
        if (norm(params.home_arm-encs.subVector(0,params.home_arm.length()-1))<1.0)
        {
            done=true;
            break;
        }
        Time::delay(0.1);
    }
    RTF_TEST_CHECK(done,"Arm has reached home!");

    drvJointHead.view(ienc);
    ienc->getAxes(&nEncs);
    encs.resize(nEncs,0.0);
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        ienc->getEncoders(encs.data());
        if (norm(encs.subVector(0,3))<1.0)
        {
            done=true;
            break;
        }
        Time::delay(0.1);
    }
    RTF_TEST_CHECK(done,"Head has reached home!");

    drvJointTorso.view(ienc);
    ienc->getAxes(&nEncs);
    encs.resize(nEncs,0.0);
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        ienc->getEncoders(encs.data());
        if (norm(encs.subVector(0,3))<1.0)
        {
            done=true;
            break;
        }
        Time::delay(0.1);
    }
    RTF_TEST_CHECK(done,"Torso has reached home!");
}


/***********************************************************************************/
void DemoRedBallTest::run()
{
    Vector pos(3,0.0);
    pos[0]=-0.4;

    pos[1]=-0.2;
    drvJointArmL.view(arm_under_test.ienc);
    drvCartArmL.view(arm_under_test.iarm);
    RTF_TEST_REPORT("Reaching with the left hand");
    testBallPosition(pos);

    pos[1]=+0.2;
    drvJointArmR.view(arm_under_test.ienc);
    drvCartArmR.view(arm_under_test.iarm);
    RTF_TEST_REPORT("Reaching with the right hand");
    testBallPosition(pos);
}

