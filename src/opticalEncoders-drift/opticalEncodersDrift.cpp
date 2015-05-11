// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#include <math.h>
<<<<<<< HEAD
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
=======
#include <TestAssert.h>
#include <Plugin.h>
>>>>>>> origin/master
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>

#include "opticalEncodersDrift.h"

//example     -v -t PositionDirect.dll -p "--robot icub --part head --joints ""(0 1 2)"" --zero 0 --frequency 0.8 --amplitude 10.0 --cycles 100 --threshold 1.0 --sampleTime 0.010 --cmdMode 2"
//example2    -v -t PositionDirect.dll -p "--robot icub --part head --joints ""(2)"" --zero 0 --frequency 0.4 --amplitude 10.0 --cycles 100 --threshold 1.0 --sampleTime 0.010 --cmdMode 0"
//            -v -t PositionDirect.dll -p "--robot icub --part head --joints ""(0)"" --zero 0 --frequency 0.8 --amplitude 10.0 --cycles 100 --threshold 1.0 --sampleTime 0.010 --cmdMode 2"
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

// prepare the plugin
PREPARE_PLUGIN(OpticalEncodersDrift)

OpticalEncodersDrift::OpticalEncodersDrift() : YarpTestCase("OpticalEncodersDrift") {
    jointsList=0;
    dd=0;
    ipos=0;
    icmd=0;
    iimd=0;
    ienc=0;
    imot=0;
    enc_jnt=0;
    enc_mot=0;
    zero_enc_mot=0;
    end_enc_mot=0;
    err_enc_mot=0;
    cycles=100;
}

OpticalEncodersDrift::~OpticalEncodersDrift() { }

bool OpticalEncodersDrift::setup(yarp::os::Property& property) {

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"), "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"), "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("zero"),    "The zero position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("max"), "The max position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("min"), "The min position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("speed"), "The positionMove reference speed must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("cycles"), "The number of cycles of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("threshold"), "The max error threshold must be given as the test parameter!");

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
<<<<<<< HEAD

=======
    
>>>>>>> origin/master
    Bottle* speedBottle = property.find("speed").asList();
    RTF_ASSERT_ERROR_IF(speedBottle!=0,"unable to parse speed parameter");

    threshold = property.find("threshold").asDouble();
    RTF_ASSERT_ERROR_IF(threshold>=0,"invalid threshold");

    cycles = property.find("cycles").asInt();
    RTF_ASSERT_ERROR_IF(cycles>=0,"invalid cycles");

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
    zero_enc_mot = new double[n_part_joints];
    end_enc_mot = new double[n_part_joints];
    err_enc_mot = new double[n_part_joints];

    max     = new double[n_cmd_joints]; for (int i=0; i< n_cmd_joints; i++) max[i]=maxBottle->get(i).asDouble();
    min     = new double[n_cmd_joints]; for (int i=0; i< n_cmd_joints; i++) min[i]=minBottle->get(i).asDouble();
    zero    = new double[n_cmd_joints]; for (int i=0; i< n_cmd_joints; i++) zero[i]=zeroBottle->get(i).asDouble();
    speed   = new double[n_cmd_joints]; for (int i=0; i< n_cmd_joints; i++) speed[i]=speedBottle->get(i).asDouble();

    return true;
}

void OpticalEncodersDrift::tearDown()
{
    if (dd) {delete dd; dd =0;}
}

void OpticalEncodersDrift::setMode(int desired_mode)
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        icmd->setControlMode((int)jointsList[i],desired_mode);
        iimd->setInteractionMode((int)jointsList[i],VOCAB_IM_STIFF);
        yarp::os::Time::delay(0.010);
    }

    int cmode;
<<<<<<< HEAD
    yarp::dev::InteractionModeEnum imode;
=======
    yarp::dev::InteractionModeEnum imode; 
>>>>>>> origin/master
    int timeout = 0;

    while (1)
    {
        int ok=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            icmd->getControlMode ((int)jointsList[i],&cmode);
            iimd->getInteractionMode((int)jointsList[i],&imode);
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

void OpticalEncodersDrift::goHome()
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipos->setRefSpeed((int)jointsList[i],speed[i]);
        ipos->positionMove((int)jointsList[i],zero[i]);
    }

    int timeout = 0;
    while (1)
    {
        int in_position=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            double tmp=0;
            ienc->getEncoder((int)jointsList[i],&tmp);
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

void OpticalEncodersDrift::run()
{
    setMode(VOCAB_CM_POSITION);
    goHome();

    bool go_to_max=false;
    int  curr_cycle=0;
    double start_time = yarp::os::Time::now();
<<<<<<< HEAD

=======
            
>>>>>>> origin/master
    imot->getMotorEncoders             (zero_enc_mot);
    while(1)
    {
        double curr_time = yarp::os::Time::now();
        double elapsed = curr_time-start_time;

        ienc->getEncoders                  (enc_jnt);
        imot->getMotorEncoders             (enc_mot);

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
<<<<<<< HEAD

=======
        
>>>>>>> origin/master
        if (elapsed >= 20.0)
        {
            RTF_ASSERT_ERROR("Timeout while moving joint");
        }

        if (reached)
        {
            if (go_to_max==false)
            {
                for (unsigned int i=0; i<jointsList.size(); i++)
                    ipos->positionMove(i,max[i]);
                go_to_max=true;
                curr_cycle++;
                start_time = yarp::os::Time::now();
            }
            else
            {
                for (unsigned int i=0; i<jointsList.size(); i++)
                    ipos->positionMove(i,min[i]);
                go_to_max=false;
                curr_cycle++;
                start_time = yarp::os::Time::now();
            }
        }

        if (curr_cycle>=cycles) break;
    }

    goHome();
    yarp::os::Time::delay(2.0);
    imot->getMotorEncoders             (end_enc_mot);

    for (int i=0; i<n_part_joints; i++)
    {
        err_enc_mot[i]=zero_enc_mot[i]-end_enc_mot[i];

        if (fabs(err_enc_mot[i]) > threshold)
        {
            //...print something
        }
    }
}
