// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <algorithm>
#include "motorEncodersSignCheck.h"
#include <yarp/manager/localbroker.h>
#include "iostream"


using namespace RTF;
using namespace RTF::YARP;
using namespace yarp::os;
using namespace yarp::dev;

// prepare the plugin
PREPARE_PLUGIN(MotorEncodersSignCheck)

MotorEncodersSignCheck::MotorEncodersSignCheck() : YarpTestCase("MotorEncodersSignCheck") {
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
    RTF_ASSERT_ERROR_IF(property.check("robot"),  "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"),   "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("home"),   "The home position must be given as the test parameter!");

    RTF_ASSERT_ERROR_IF(property.check("pwmStep"),    "The output_step must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("pwmMax"),     "The output_max must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* homeBottle = property.find("home").asList();
    RTF_ASSERT_ERROR_IF(homeBottle!=0,"unable to parse zero parameter");

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* pwm_step_Bottle = property.find("pwmStep").asList();
    RTF_ASSERT_ERROR_IF(pwm_step_Bottle!=0,"unable to parse pwmStep parameter");

    Bottle* command_delay_Bottle = property.find("commandDelay").asList();
    if(command_delay_Bottle==0)
        RTF_TEST_REPORT("Command delay not configured. default value (0.1 sec) will be used ");


    Bottle* pwm_max_Bottle = property.find("pwmMax").asList();
    RTF_ASSERT_ERROR_IF(pwm_max_Bottle!=0,"unable to parse joints parameter");

    Bottle* threshold_Bottle = property.find("PosThreshold").asList();
    if(threshold_Bottle==0)
        RTF_TEST_REPORT("Position threshold not configured. default value (5 deg) will be used ");

    Bottle* pwm_start_Bottle = property.find("pwmStart").asList();
    RTF_ASSERT_ERROR_IF(pwm_start_Bottle!=0,"unable to parse pwmStart parameter");


    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/mEncSignCheckTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(iopl),"Unable to open openloop interface");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(imenc),"Unable to open interaction mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipid),"Unable to open ipidcontrol interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
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

    jPosMotion = new jointsPosMotion(dd, jointsList);
    jPosMotion->setTolerance(2.0);
    jPosMotion->setTimeout(10); //10 sec


    return true;
}

void MotorEncodersSignCheck::tearDown()
{
    char buff[500];
    sprintf(buff,"Closing test module");RTF_TEST_REPORT(buff);
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

    iopl->setRefOutput((int)jointsList[i],opl);
    double last_opl_cmd=yarp::os::Time::now();
    yarp::os::Time::delay(3.0); //i need to wait a while because when i set ref output zero, joint may move (due to stiction or gravity) and I should save the position when pwm=0

    imenc->getMotorEncoder((int)jointsList[i],&start_enc);


    while (not_moving)
    {

        iopl->setRefOutput((int)jointsList[i],opl);
        imenc->getMotorEncoder((int)jointsList[i],&enc);

        if(enc > start_enc+pos_threshold[i])
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            sprintf(buff,"TEST SUCCESS (pwm=%f) enc=%f start_enc=%f",opl, enc, start_enc);
            RTF_TEST_REPORT(buff);
        }
        else if(enc < start_enc-pos_threshold[i])
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            RTF_TEST_FAIL_IF(0, RTF::Asserter::format("because enc readings drecrease enc=%f start_enc=%f (output=%f)", enc, start_enc, opl));
        }
        else if (jPosMotion->checkJointLimitsReached((int)jointsList[i]))
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            RTF_TEST_FAIL_IF(0,RTF::Asserter::format("Test failed because hw limit was touched (enc=%f)",enc));
        }

        if (yarp::os::Time::now()-last_opl_cmd>opl_delay[i])
        {
            opl+=opl_step[i];
            last_opl_cmd=yarp::os::Time::now();
        }
        if (opl>=opl_max[i])
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            RTF_TEST_FAIL_IF(0,RTF::Asserter::format("Test failed failed because max output was reached(output=%f)",opl));
        }

        yarp::os::Time::delay(0.010);

        if (time-time_old>5.0 && not_moving==true)
        {
            sprintf(buff,"test in progress on joint %d, current output value = %f",(int)jointsList[i],opl);RTF_TEST_REPORT(buff);
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
        RTF_TEST_FAIL_IF((ipid->getOutput((int)jointsList[i], &posout)),
                         RTF::Asserter::format(" getOutput j %d return false",(int)jointsList[i]));

        setModeSingle(i,VOCAB_CM_OPENLOOP,VOCAB_IM_STIFF);
        iopl->setRefOutput((int)jointsList[i],0.0);

        RTF_TEST_REPORT(RTF::Asserter::format("Testing joint %d with starting pwm = %.2f. In position j had pwm = %.2f",(int)jointsList[i], opl_start[i], posout));
        OplExecute(i);

        jPosMotion->setAndCheckPosControlMode();
        jPosMotion->goTo(home);

    }

 }
