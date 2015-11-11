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

    RTF_ASSERT_ERROR_IF(property.check("outputStep"),    "The output_step must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("outputDelay") ,  "The output_delay must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("outputMax"),     "The output_max must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* homeBottle = property.find("home").asList();
    RTF_ASSERT_ERROR_IF(homeBottle!=0,"unable to parse zero parameter");

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* output_step_Bottle = property.find("outputStep").asList();
    RTF_ASSERT_ERROR_IF(output_step_Bottle!=0,"unable to parse joints parameter");

    Bottle* output_delay_Bottle = property.find("outputDelay").asList();
    RTF_ASSERT_ERROR_IF(output_delay_Bottle!=0,"unable to parse joints parameter");

    Bottle* output_max_Bottle = property.find("outputMax").asList();
    RTF_ASSERT_ERROR_IF(output_max_Bottle!=0,"unable to parse joints parameter");

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

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
    jointsList.clear();
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    home.resize (n_cmd_joints);               for (int i=0; i< n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    opl_step.resize (n_cmd_joints);           for (int i=0; i< n_cmd_joints; i++) opl_step[i]=output_step_Bottle->get(i).asDouble();
    opl_delay.resize (n_cmd_joints);          for (int i=0; i< n_cmd_joints; i++) opl_delay[i]=output_delay_Bottle->get(i).asDouble();
    opl_max.resize (n_cmd_joints);            for (int i=0; i< n_cmd_joints; i++) opl_max[i]=output_max_Bottle->get(i).asDouble();

    jPosMotion = new jointsPosMotion(dd, jointsList);
    jPosMotion->setTolerance(2.0);
    jPosMotion->setTimeout(10); //10 sec

    return true;
}

void MotorEncodersSignCheck::tearDown()
{
    char buff[500];
    sprintf(buff,"Closing test module");RTF_TEST_REPORT(buff);
    jPosMotion->setAndCheckPosControlMode();
    jPosMotion->goTo(home);

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
    double opl=0;

    iopl->setRefOutput((int)jointsList[i],opl);
    double last_opl_cmd=yarp::os::Time::now();
    yarp::os::Time::delay(3.0); //i need to wait a while because when i set ref output zero, joint may move (due to stiction or gravity) and I should save the position when pwm=0

    imenc->getMotorEncoder((int)jointsList[i],&start_enc);


    while (not_moving)
    {

        iopl->setRefOutput((int)jointsList[i],opl);
        imenc->getMotorEncoder((int)jointsList[i],&enc);

        if(enc > start_enc+delta)
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            sprintf(buff,"Test success (output=%f) enc=%f start_enc=%f",opl, enc, start_enc);
            RTF_TEST_REPORT(buff);
        }
        else if(enc < start_enc)
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            sprintf(buff,"Test failed because enc readings drecrease (output=%f)",opl);
            RTF_TEST_REPORT(buff);
        }
        else if (opl>=opl_max[i])
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            sprintf(buff,"Test failed failed because max output was reached(output=%f)",opl);
            RTF_TEST_REPORT(buff);
        }
        else if (jPosMotion->checkJointLimitsReached((int)jointsList[i]))
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            sprintf(buff,"Test failed because hw limit was touched (enc=%f)",enc);
            RTF_TEST_REPORT(buff);
        }

        if (yarp::os::Time::now()-last_opl_cmd>opl_delay[i])
        {
            opl+=opl_step[i];
            last_opl_cmd=yarp::os::Time::now();
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
        setModeSingle(i,VOCAB_CM_OPENLOOP,VOCAB_IM_STIFF);
        iopl->setRefOutput((int)jointsList[i],0.0);

        sprintf(buff,"Testing joint %d",(int)jointsList[i]);
        RTF_TEST_REPORT(buff);
        OplExecute(i);

        jPosMotion->setAndCheckPosControlMode();
        jPosMotion->goTo(home);

    }

 }
