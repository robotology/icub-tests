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

#include <memory>
#include <algorithm>
#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/filters.h>

#include "DemoRedBallTest.h"

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(DemoRedBallTest)


/***********************************************************************************/
DemoRedBallTest::DemoRedBallTest() : yarp::robottestingframework::TestCase("DemoRedBallTest")
{
}


/***********************************************************************************/
DemoRedBallTest::~DemoRedBallTest()
{
}


/***********************************************************************************/
bool DemoRedBallTest::setup(Property &property)
{
    Time::useNetworkClock("/clock");

    string context=property.check("context",Value("demoRedBall")).asString();
    string from=property.check("from",Value("config-test.ini")).asString();

    // retrieve demoRedBall parameters
    ResourceFinder rf;
    rf.setDefaultContext(context.c_str());
    rf.setDefaultConfigFile(from.c_str());
    rf.configure(0,NULL);

    // fallback values
    params.robot="icubSim";
    params.eye="left";
    params.reach_tol=0.01;
    params.use_torso=true;
    params.use_left=true;
    params.use_right=true;
    params.home_arm.resize(7,0.0);

    Bottle &general=rf.findGroup("general");
    if (!general.isNull())
    {
        params.robot=general.check("robot",Value(params.robot)).asString();
        params.eye=general.check("eye",Value(params.eye)).asString();
        params.reach_tol=general.check("reach_tol",Value(params.reach_tol)).asFloat64();
        params.use_torso=(general.check("torso",Value(params.use_torso?"on":"off")).asString()=="on");
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
                params.home_arm[i]=poss.get(1+i).asFloat64();
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

    if (params.use_torso)
    {
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote",("/"+params.robot+"/"+"torso"));
        optJoint.put("local",("/"+getName()+"/joint/torso"));

        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(drvJointTorso.open(optJoint),
                            "Unable to open clients for torso!");
    }

    rpcPort.open("/"+getName()+"/rpc");
    string dest="/demoRedBall/rpc";
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(Network::connect(rpcPort.getName(),dest),
                            Asserter::format("Unable to connect to %s!",dest.c_str()));

    guiPort.open("/"+getName()+"/gui:i");
    string src="/demoRedBall/gui:o";
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(Network::connect(src,guiPort.getName()),
                            Asserter::format("Unable to connect to %s!",src.c_str()));                            

    return true;
}


/***********************************************************************************/
void DemoRedBallTest::tearDown()
{
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Closing Clients");
    rpcPort.close();
    guiPort.close();
    if (params.use_left)
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvJointArmL.close()&&drvCartArmL.close(),
                           "Unable to close client for left_arm!");
    }
    if (params.use_right)
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvJointArmR.close()&&drvCartArmR.close(),
                           "Unable to close client for right_arm!");
    }
    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvJointHead.close()&&drvGaze.close(),
                       "Unable to close client for head!");
    if (params.use_torso)
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvJointTorso.close(),"Unable to close client for left_arm!");
    }
}


/***********************************************************************************/
bool DemoRedBallTest::getBallPosition(const Bottle* b, Vector& pos)
{
    if (b->size()>=15)
    {
        if (b->get(0).isString() && (b->get(0).asString()=="object"))
        {
            pos.resize(3);
            pos[0]=b->get(5).asFloat64()/1000.;
            pos[1]=b->get(6).asFloat64()/1000.;
            pos[2]=b->get(7).asFloat64()/1000.;
            return true;
        }
    }
    return false;
}


/***********************************************************************************/
void DemoRedBallTest::testBallPosition(const Vector &dpos)
{
    Vector x,o,encs;
    int nEncs; IEncoders* ienc;
    bool done=false;
    double t0;

    Bottle cmd,rep;
    cmd.addString("update_pose");
    cmd.addFloat64(dpos[0]);
    cmd.addFloat64(dpos[1]);
    cmd.addFloat64(dpos[2]);
    rpcPort.write(cmd,rep);

    Time::delay(3.0);

    cmd.clear();
    cmd.addString("start");
    cmd.addFloat64(0.);
    cmd.addFloat64(-50.);
    cmd.addFloat64(10.);
    rpcPort.write(cmd,rep);

    auto filt = make_unique<MedianFilter>(5, Vector{0., 0., 0.});

    IGazeControl* igaze;
    drvGaze.view(igaze);

    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        if (auto* gui=guiPort.read(false))
        {
            Vector pos;
            if (getBallPosition(gui,pos))
            {
                filt->filt(pos);
            }
        }
        igaze->getFixationPoint(x);
        if (norm(filt->output()-x)<2.0*params.reach_tol)
        {
            done=true;
            break;
        }
        Time::delay(0.01);
    }
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Ball gazed at with the eyes!");

    filt->init(Vector{0., 0., 0.});
    done=false;
    t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        if (auto* gui=guiPort.read(false))
        {
            Vector pos;
            if (getBallPosition(gui,pos))
            {
                filt->filt(pos);
            }
        }
        arm_under_test.iarm->getPose(x,o);
        if (norm(filt->output()-x)<params.reach_tol)
        {
            done=true;
            break;
        }
        Time::delay(0.01);
    }
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Ball reached with the hand!");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Going home");
    cmd.clear();
    cmd.addString("stop");
    rpcPort.write(cmd,rep);

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

    if (params.use_torso)
    {
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
}


/***********************************************************************************/
void DemoRedBallTest::run()
{    
    if (params.use_torso || params.use_left)
    {
        Vector dpos(3,0.0);
        dpos[1]=-0.06;
        dpos[2]=-0.3;
        drvJointArmL.view(arm_under_test.ienc);
        drvCartArmL.view(arm_under_test.iarm);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Reaching with the left hand");
        testBallPosition(dpos);
    }

   if (params.use_torso || params.use_right)
    {
        Vector dpos(3,0.0);
        dpos[1]=+0.06;
        dpos[2]=-0.3;
        drvJointArmR.view(arm_under_test.ienc);
        drvCartArmR.view(arm_under_test.iarm);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Reaching with the right hand");
        testBallPosition(dpos);
    }
}

