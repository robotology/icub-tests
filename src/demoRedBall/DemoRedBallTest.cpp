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

#include <algorithm>
#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include "DemoRedBallTest.h"

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(DemoRedBallTest)


/***********************************************************************************/
class DemoRedBallPosition : public PeriodicThread
{
    string name;
    IGazeControl *igaze;
    string eye;
    Vector pos;
    bool visible;
    BufferedPort<Bottle> port;

    bool threadInit()
    {
        string dest="/demoRedBall/trackTarget:i";
        port.open(("/"+name+"/redballpos:o"));
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(Network::connect(port.getName(),dest,"udp"),
                            Asserter::format("Unable to connect to %s!",dest.c_str()));
        return true;
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
            Vector pos_=SE3inv(T)*pos;

            Bottle &cmd=port.prepare();
            cmd.clear();
            cmd.addDouble(pos_[0]);
            cmd.addDouble(pos_[1]);
            cmd.addDouble(pos_[2]);
            cmd.addDouble(0.0);
            cmd.addDouble(0.0);
            cmd.addDouble(0.0);
            cmd.addDouble(visible?1.0:0.0);
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
                        PeriodicThread(0.1), name(name_),
                        eye(eye_), pos(4,0.0),
                        visible(false)
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

    void setVisible()   { visible=true;  }
    void setInvisible() { visible=false; }
};


/***********************************************************************************/
DemoRedBallTest::DemoRedBallTest() : yarp::robottestingframework::TestCase("DemoRedBallTest"), redBallPos(NULL)
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
    string from=property.check("from",Value("config-test.ini")).asString();

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

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Opening Clients");
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

        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(drvJointArmL.open(optJoint)&&drvCartArmL.open(optCart),
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

        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(drvJointArmR.open(optJoint)&&drvCartArmR.open(optCart),
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

        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(drvJointHead.open(optJoint)&&drvGaze.open(optGaze),
                            "Unable to open clients for head!");
    }

    {
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote",("/"+params.robot+"/"+"torso"));
        optJoint.put("local",("/"+getName()+"/joint/torso"));

        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(drvJointTorso.open(optJoint),
                            "Unable to open clients for torso!");
    }

    redBallPos=new DemoRedBallPosition(getName(),drvGaze,params.eye);
    redBallPos->start();

    return true;
}


/***********************************************************************************/
void DemoRedBallTest::tearDown()
{
    redBallPos->stop();

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Closing Clients");
    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvJointArmL.close()&&drvCartArmL.close(),
                       "Unable to close client for left_arm!");
    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvJointArmR.close()&&drvCartArmR.close(),
                       "Unable to close client for right_arm!");
    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvJointHead.close()&&drvGaze.close(),
                       "Unable to close client for head!");
    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvJointTorso.close(),"Unable to close client for left_arm!");
}


/***********************************************************************************/
void DemoRedBallTest::testBallPosition(const Vector &pos)
{
    DemoRedBallPosition *ball=dynamic_cast<DemoRedBallPosition*>(redBallPos);
    ball->setPos(pos);
    ball->setVisible();

    Vector x,o,encs;
    int nEncs; IEncoders* ienc;
    bool done=false;
    double t0;

    IGazeControl* igaze;
    drvGaze.view(igaze);
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        igaze->getFixationPoint(x);
        if (norm(pos-x)<2.0*params.reach_tol)
        {
            done=true;
            break;
        }
        Time::delay(0.01);
    }
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Ball gazed at with the eyes!");

    done=false;
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        arm_under_test.iarm->getPose(x,o);
        if (norm(pos-x)<params.reach_tol)
        {
            done=true;
            break;
        }
        Time::delay(0.01);
    }
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Ball reached with the hand!");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Going home");
    ball->setInvisible();

    arm_under_test.ienc->getAxes(&nEncs);
    encs.resize(nEncs,0.0);
    done=false;
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        arm_under_test.ienc->getEncoders(encs.data());
        if (norm(params.home_arm-encs.subVector(0,params.home_arm.length()-1))<5.0)
        {
            done=true;
            break;
        }
        Time::delay(1.0);
    }
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Arm has reached home!");

    drvJointHead.view(ienc);
    ienc->getAxes(&nEncs);
    encs.resize(nEncs,0.0);
    done=false;
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        ienc->getEncoders(encs.data());
        if (norm(encs.subVector(0,3))<5.0)
        {
            done=true;
            break;
        }
        Time::delay(1.0);
    }
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Head has reached home!");

    drvJointTorso.view(ienc);
    ienc->getAxes(&nEncs);
    encs.resize(nEncs,0.0);
    done=false;
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        ienc->getEncoders(encs.data());
        if (norm(encs.subVector(0,3))<5.0)
        {
            done=true;
            break;
        }
        Time::delay(1.0);
    }
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Torso has reached home!");
}


/***********************************************************************************/
void DemoRedBallTest::run()
{
    Vector pos(3,0.0);
    pos[0]=-0.3;

    pos[1]=-0.15;
    drvJointArmL.view(arm_under_test.ienc);
    drvCartArmL.view(arm_under_test.iarm);
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Reaching with the left hand");
    testBallPosition(pos);

    pos[1]=+0.15;
    drvJointArmR.view(arm_under_test.ienc);
    drvCartArmR.view(arm_under_test.iarm);
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Reaching with the right hand");
    testBallPosition(pos);
}

