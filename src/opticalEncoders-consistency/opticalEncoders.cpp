// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <math.h>
#include <TestAssert.h>
#include <Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>

#include "opticalEncoders.h"

//example     -v -t PositionDirect.dll -p "--robot icub --part head --joints ""(0 1 2)"" --zero 0 --frequency 0.8 --amplitude 10.0 --cycles 10 --tolerance 1.0 --sampleTime 0.010 --cmdMode 2"
//example2    -v -t PositionDirect.dll -p "--robot icub --part head --joints ""(2)"" --zero 0 --frequency 0.4 --amplitude 10.0 --cycles 10 --tolerance 1.0 --sampleTime 0.010 --cmdMode 0"
//            -v -t PositionDirect.dll -p "--robot icub --part head --joints ""(0)"" --zero 0 --frequency 0.8 --amplitude 10.0 --cycles 10 --tolerance 1.0 --sampleTime 0.010 --cmdMode 2"
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

// prepare the plugin
PREPARE_PLUGIN(OpticalEncoders)

OpticalEncoders::OpticalEncoders() : YarpTestCase("OpticalEncoders") {
    jointsList=0;
    dd=0;
    ipos=0;
    icmd=0;
    iimd=0;
    ienc=0;
    imot=0;
    enc_jnt=0;
    enc_jnt2mot=0;
    enc_mot=0;
    spd_jnt=0;
    spd_jnt2mot=0;
    spd_mot=0;
    acc_jnt=0;
    acc_jnt2mot=0;
    acc_mot=0;

    matrix_arms.resize(16,16);
    matrix_arms.eye();
    double r=40;
    double R=65;
    matrix_arms (0,0) = 1;
    matrix_arms (1,0) = -R/r;
    matrix_arms (1,1) = R/r;
    matrix_arms (2,0) = -R/r;
    matrix_arms (2,1) = R/r;
    matrix_arms (2,2) = R/r;

    matrix_torso.resize(3,3);
    matrix_torso.eye();
    r=0.022*1000;
    R=0.04*1000;
    matrix_torso (0,0) = R/r;
    matrix_torso (1,0) = 0;
    matrix_torso (2,0) = 0;
    matrix_torso (0,1) = -1;
    matrix_torso (1,1) = 1;
    matrix_torso (2,1) = 1;
    matrix_torso (0,2) = 0;
    matrix_torso (1,2) = -1;
    matrix_torso (2,2) = 1;

    matrix_legs.resize(6,6);
    matrix_legs.eye();
    matrix_legs (0,0) = 60/40;

    matrix_head.resize(6,6);
    matrix_head.eye();
}

OpticalEncoders::~OpticalEncoders() { }

bool OpticalEncoders::setup(yarp::os::Property& property) {

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"), "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"), "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("zero"),    "The zero position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("max"), "The frequency of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("min"), "The amplitude of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("speed"), "The number of cycles of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("tolerance"), "The tolerance of the control signal must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* zeroBottle = property.find("zero").asList();
    RTF_ASSERT_ERROR_IF(zeroBottle!=0,"unable to parse zero parameter");

    Bottle* maxBottle = property.find("max").asList();
    RTF_ASSERT_ERROR_IF(maxBottle!=0,"unable to parse max parameter");

    Bottle* minBottle = property.find("min").asList();
    RTF_ASSERT_ERROR_IF(minBottle!=0,"unable to parse min parameter");
    
    Bottle* speedBottle = property.find("speed").asList();
    RTF_ASSERT_ERROR_IF(speedBottle!=0,"unable to parse speed parameter");

    tolerance = property.find("tolerance").asDouble();
    RTF_ASSERT_ERROR_IF(tolerance>=0,"invalid tolerance");

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/positionDirectTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(imot),"Unable to open motor encoders interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    enc_jnt = new double[n_part_joints];
    enc_mot = new double[n_part_joints];
    spd_jnt = new double[n_part_joints];
    spd_mot = new double[n_part_joints];
    acc_jnt = new double[n_part_joints];
    acc_mot = new double[n_part_joints];

    max     = new double[n_cmd_joints]; for (int i=0; i< n_cmd_joints; i++) max[i]=maxBottle->get(i).asDouble();
    min     = new double[n_cmd_joints]; for (int i=0; i< n_cmd_joints; i++) min[i]=minBottle->get(i).asDouble();
    zero    = new double[n_cmd_joints]; for (int i=0; i< n_cmd_joints; i++) zero[i]=zeroBottle->get(i).asDouble();
    speed   = new double[n_cmd_joints]; for (int i=0; i< n_cmd_joints; i++) speed[i]=speedBottle->get(i).asDouble();

    return true;
}

void OpticalEncoders::tearDown()
{
    if (dd) {delete dd; dd =0;}
}

void OpticalEncoders::setMode(int desired_mode)
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        icmd->setControlMode(jointsList[i],desired_mode);
        iimd->setInteractionMode(jointsList[i],VOCAB_IM_STIFF);
        yarp::os::Time::delay(0.010);
    }

    int cmode;
    yarp::dev::InteractionModeEnum imode; 
    int timeout = 0;

    while (1)
    {
        int ok=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            icmd->getControlMode (jointsList[i],&cmode);
            iimd->getInteractionMode(jointsList[i],&imode);
            if (cmode==desired_mode && imode==VOCAB_IM_STIFF) ok++;
        }
        if (ok==jointsList.size()) break;
        if (timeout>100)
        {
            RTF_ASSERT_ERROR("Unable to set control mode/interaction mode");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

void OpticalEncoders::goHome()
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipos->setRefSpeed(jointsList[i],speed[i]);
        ipos->positionMove(jointsList[i],zero[i]);
    }

    int timeout = 0;
    while (1)
    {
        int in_position=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            double tmp=0;
            ienc->getEncoder(jointsList[i],&tmp);
            if (fabs(tmp-zero[i])<0.5) in_position++;
        }
        if (in_position==jointsList.size()) break;
        if (timeout>100)
        {
            RTF_ASSERT_ERROR("Timeout while reaching zero position");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

void OpticalEncoders::run()
{
    setMode(VOCAB_CM_POSITION);
    goHome();

    bool go_to_max=false;
    int  cycles=0;
    double start_time = yarp::os::Time::now();
    
    if      (partName=="left_arm")  {matrix = matrix_arms;}
    else if (partName=="right_arm") {matrix = matrix_arms;}
    else if (partName=="torso")     {matrix = matrix_torso;}
    else if (partName=="left_leg")  {matrix = matrix_legs;}
    else if (partName=="right_leg") {matrix = matrix_legs;}
    else if (partName=="head")      {matrix = matrix_head;}
    else
    {
        RTF_ASSERT_ERROR("Unknown part name, missing matrix for this part");
    }
    trasp_matrix = matrix.transposed();
    inv_matrix = yarp::math::luinv(matrix);
    inv_trasp_matrix = inv_matrix.transposed();

    while(1)
    {
        double curr_time = yarp::os::Time::now();
        double elapsed = curr_time-start_time;

        ienc->getEncoders                  (enc_jnt);
        imot->getMotorEncoders             (enc_mot);
        ienc->getEncoderSpeeds             (spd_jnt);
        imot->getMotorEncoderSpeeds        (spd_mot);
        ienc->getEncoderAccelerations      (acc_jnt);
        imot->getMotorEncoderAccelerations (acc_mot);
        enc_jnt2mot = matrix * yarp::sig::Vector(n_part_joints,enc_jnt);
        spd_jnt2mot = matrix * yarp::sig::Vector(n_part_joints,spd_jnt);
        acc_jnt2mot = matrix * yarp::sig::Vector(n_part_joints,acc_jnt);
        enc_jnt2mot = enc_jnt2mot * 100;
        spd_jnt2mot = spd_jnt2mot * 100;
        acc_jnt2mot = acc_jnt2mot * 100;

        bool reached= false;
        int in_position=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            double curr_val=0;
            if (go_to_max==false) curr_val = min[i];
            else                  curr_val = max[i];
            if (fabs(enc_jnt[i]-curr_val)<0.5) in_position++;
        }
        if (in_position==jointsList.size()) reached=true;
        
        if (elapsed >= 20.0)
        {
            RTF_ASSERT_ERROR("Timeout while moving joint");
        }

        if (reached)
        {
            if (go_to_max==false)
            {
                for (int i=0; i<jointsList.size(); i++)
                    ipos->positionMove(i,max[i]);
                go_to_max=true;
                cycles++;
                start_time = yarp::os::Time::now();
            }
            else
            {
                for (int i=0; i<jointsList.size(); i++)
                    ipos->positionMove(i,min[i]);
                go_to_max=false;
                cycles++;
                start_time = yarp::os::Time::now();
            }
        }

        if (cycles>=30) break;
    }

    goHome();
}
