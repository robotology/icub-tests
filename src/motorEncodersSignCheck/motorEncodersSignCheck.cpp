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


#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <algorithm>
#include "motorEncodersSignCheck.h"
#include <yarp/manager/localbroker.h>
#include "iostream"


using namespace robottestingframework;

using namespace yarp::os;
using namespace yarp::dev;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(MotorEncodersSignCheck)

MotorEncodersSignCheck::MotorEncodersSignCheck() : yarp::robottestingframework::TestCase("MotorEncodersSignCheck") {
    jointsList=0;
    dd=0;
    icmd=0;
    iimd=0;
    ienc=0;
    imenc=0;
    jPosMotion=0;
}

MotorEncodersSignCheck::~MotorEncodersSignCheck() { }

bool MotorEncodersSignCheck::setup(yarp::os::Property& property) {

    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("robot"),  "The robot name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("part"),   "The part name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("joints"), "The joints list must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("home"),   "The home position must be given as the test parameter!");

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("pwmStep"),    "The output_step must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("pwmMax"),     "The output_max must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* homeBottle = property.find("home").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(homeBottle!=0,"unable to parse zero parameter");

    Bottle* jointsBottle = property.find("joints").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* pwm_step_Bottle = property.find("pwmStep").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(pwm_step_Bottle!=0,"unable to parse pwmStep parameter");

    Bottle* command_delay_Bottle = property.find("commandDelay").asList();
    if(command_delay_Bottle==0)
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Command delay not configured. default value (0.1 sec) will be used ");


    Bottle* pwm_max_Bottle = property.find("pwmMax").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(pwm_max_Bottle!=0,"unable to parse joints parameter");

    Bottle* threshold_Bottle = property.find("PosThreshold").asList();
    if(threshold_Bottle==0)
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Position threshold not configured. default value (5 deg) will be used ");

    Bottle* pwm_start_Bottle = property.find("pwmStart").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(pwm_start_Bottle!=0,"unable to parse pwmStart parameter");


    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/mEncSignCheckTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->isValid(),"Unable to open device driver");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ipwm),"Unable to open pwm interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ienc),"Unable to open encoders interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(icmd),"Unable to open control mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iimd),"Unable to open interaction mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(imenc),"Unable to open interaction mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ipid),"Unable to open ipidcontrol interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
    jointsList.clear();
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    home.resize (n_cmd_joints);               for (int i=0; i< n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    opl_step.resize (n_cmd_joints);           for (int i=0; i< n_cmd_joints; i++) opl_step[i]=pwm_step_Bottle->get(i).asDouble();
    opl_max.resize (n_cmd_joints);            for (int i=0; i< n_cmd_joints; i++) opl_max[i]=pwm_max_Bottle->get(i).asDouble();
    opl_start.resize(n_cmd_joints);           for (int i=0; i< n_cmd_joints; i++) opl_start[i]=pwm_start_Bottle->get(i).asDouble();
    pos_threshold.resize (n_cmd_joints);
    if(threshold_Bottle!=0)
    {
        for (int i=0; i< n_cmd_joints; i++)
            pos_threshold[i]=threshold_Bottle->get(i).asDouble();
    }
    else
    {
        for (int i=0; i< n_cmd_joints; i++)
            pos_threshold[i]=5;
    }

    opl_delay.resize (n_cmd_joints);
    if(command_delay_Bottle!=0)
    {
        for (int i=0; i< n_cmd_joints; i++)
            opl_delay[i]=command_delay_Bottle->get(i).asDouble();
    }
    else
    {
        for (int i=0; i< n_cmd_joints; i++)
            opl_delay[i]=0.1;
    }

    jPosMotion = new yarp::robottestingframework::jointsPosMotion(dd, jointsList);
    jPosMotion->setTolerance(2.0);
    jPosMotion->setTimeout(10); //10 sec


    return true;
}

void MotorEncodersSignCheck::tearDown()
{
    char buff[500];
    sprintf(buff,"Closing test module");ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);
    if(jPosMotion)
    {
        jPosMotion->setAndCheckPosControlMode();
        jPosMotion->goTo(home);
    }
    delete jPosMotion;

    if (dd) {delete dd; dd =0;}
}


void MotorEncodersSignCheck::setModeSingle(int i, int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode)
{
    icmd->setControlMode((int)jointsList[i],desired_control_mode);
    iimd->setInteractionMode((int)jointsList[i],desired_interaction_mode);
    yarp::os::Time::delay(0.010);
}

void MotorEncodersSignCheck::OplExecute(int i)
{
    char buff[500];
    double time     = yarp::os::Time::now();
    double time_old = yarp::os::Time::now();
    double enc=0;
    double start_enc=0;
    double const delta = 10;
    bool not_moving = true;
    double opl=opl_start[i];

    ipwm->setRefDutyCycle((int)jointsList[i], opl);
    double last_opl_cmd=yarp::os::Time::now();
    yarp::os::Time::delay(3.0); //i need to wait a while because when i set ref output zero, joint may move (due to stiction or gravity) and I should save the position when pwm=0

    imenc->getMotorEncoder((int)jointsList[i],&start_enc);


    while (not_moving)
    {

        ipwm->setRefDutyCycle(jointsList[i], opl);
        imenc->getMotorEncoder((int)jointsList[i],&enc);

        if(enc > start_enc+pos_threshold[i])
        {
            ipwm->setRefDutyCycle((int)jointsList[i], 0.0);
            not_moving=false;
            sprintf(buff,"TEST SUCCESS (pwm=%f) enc=%f start_enc=%f",opl, enc, start_enc);
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);
        }
        else if(enc < start_enc-pos_threshold[i])
        {
            ipwm->setRefDutyCycle((int)jointsList[i], 0.0);
            not_moving=false;
            ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(0, robottestingframework::Asserter::format("because enc readings drecrease enc=%f start_enc=%f (output=%f)", enc, start_enc, opl));
        }
        else if (jPosMotion->checkJointLimitsReached((int)jointsList[i]))
        {
            ipwm->setRefDutyCycle((int)jointsList[i], 0.0);
            not_moving=false;
            ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(0,robottestingframework::Asserter::format("Test failed because hw limit was touched (enc=%f)",enc));
        }

        if (yarp::os::Time::now()-last_opl_cmd>opl_delay[i])
        {
            opl+=opl_step[i];
            last_opl_cmd=yarp::os::Time::now();
        }
        if (opl>=opl_max[i])
        {
            ipwm->setRefDutyCycle((int)jointsList[i], 0.0);
            not_moving=false;
            ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(0,robottestingframework::Asserter::format("Test failed failed because max output was reached(output=%f)",opl));
        }

        yarp::os::Time::delay(0.010);

        if (time-time_old>5.0 && not_moving==true)
        {
            sprintf(buff,"test in progress on joint %d, current output value = %f",(int)jointsList[i],opl);ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);
            time_old=time;
        }
    }
}

void MotorEncodersSignCheck::run()
{

    char buff[500];
    jPosMotion->setAndCheckPosControlMode();
    jPosMotion->goTo(home);


    for (unsigned int i=0 ; i<jointsList.size(); i++)
    {
        double posout=0;
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE((ipid->getPidOutput(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, (int)jointsList[i], &posout)),
                         robottestingframework::Asserter::format(" getOutput j %d return false",(int)jointsList[i]));

        setModeSingle(i,VOCAB_CM_PWM,VOCAB_IM_STIFF);
        ipwm->setRefDutyCycle((int)jointsList[i],0.0);

        ROBOTTESTINGFRAMEWORK_TEST_REPORT(robottestingframework::Asserter::format("Testing joint %d with starting pwm = %.2f. In position j had pwm = %.2f",(int)jointsList[i], opl_start[i], posout));
        OplExecute(i);

        jPosMotion->setAndCheckPosControlMode();
        jPosMotion->goTo(home);

    }

 }
