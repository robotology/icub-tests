// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#include <TestAssert.h>
#include <Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include "PositionDirect.h"

using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;

// prepare the plugin
PREPARE_PLUGIN(PositionDirect)

PositionDirect::PositionDirect() : YarpTestCase("PositionDirect") {
    jointsList=0;
    currPos=0;
    dd=0;
    ipos=0;
    icmd=0;
    iimd=0;
    ienc=0;
    idir=0;
}

PositionDirect::~PositionDirect() { }

bool PositionDirect::setup(yarp::os::Property& property) {

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"), "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"), "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("zero"),    "The zero position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("frequency"), "The frequency of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("amplitude"), "The amplitude of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("cycles"), "The number of cycles of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("tolerance"), "The tolerance of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("sampleTime"), "The tolerance of the control signal must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"invalid joints parameter");
    njoints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(njoints>0,"invalid number of joints");
    jointsList=new double[njoints];
    currPos=new double[njoints];
    for (int i=0; i <njoints; i++) jointsList[i]=jointsBottle->get(i).asDouble();
    
    frequency = property.find("frequency").asDouble();
    RTF_ASSERT_ERROR_IF(frequency>0,"invalid frequency");

    amplitude = property.find("amplitude").asDouble();
    RTF_ASSERT_ERROR_IF(amplitude>0,"invalid amplitude");

    zero = property.find("zero").asDouble();

    cycles = property.find("cycles").asDouble();
    RTF_ASSERT_ERROR_IF(cycles>0,"invalid cycles");

    tolerance = property.find("tolerance").asDouble();
    RTF_ASSERT_ERROR_IF(tolerance>0,"invalid tolerance");

    sampleTime = property.find("sampleTime").asDouble();
    RTF_ASSERT_ERROR_IF(sampleTime>0,"invalid sampleTime");

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/positionDirectTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(idir),"Unable to open position direct interface");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");

    return true;
}

void PositionDirect::tearDown()
{
    if (jointsList) {delete jointsList; jointsList =0;}
    if (dd) {delete dd; dd =0;}
}

void PositionDirect::goHome()
{
    for (int i=0; i<njoints; i++)
    {
        icmd->setControlMode(i,VOCAB_CM_POSITION);
        iimd->setInteractionMode(i,VOCAB_IM_STIFF);
        yarp::os::Time::delay(0.100);
    }

    for (int i=0; i<njoints; i++)
    {
        int cmode=0;
        yarp::dev::InteractionModeEnum imode;
        icmd->getControlMode(i, &cmode);
        iimd->getInteractionMode(i, &imode);
        RTF_ASSERT_ERROR_IF(cmode==VOCAB_CM_POSITION,"Unable to set control mode: VOCAB_CM_POSITION");
        RTF_ASSERT_ERROR_IF(imode==VOCAB_IM_STIFF,"Unable to set interaction mode: VOCAB_IM_STIFF");
    }

    for (int i=0; i<njoints; i++)
    {
        ipos->positionMove(i,zero);
    }

    int timeout = 0;
    while (1)
    {
        ienc->getEncoders(currPos);
        int in_position=0;
        for (int i=0; i<njoints; i++)
        {
            if (fabs(currPos[i]-zero)<1) in_position++;
        }
        if (in_position==njoints) break;
        RTF_ASSERT_ERROR_IF(timeout>100,"Timeout while reaching zero position");
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

void PositionDirect::run()
{
    goHome();

    icmd->setControlMode(jointsList[0],VOCAB_CM_POSITION_DIRECT);
    yarp::os::Time::delay(0.100);

    int cmode=0;
    icmd->getControlMode(0, &cmode);
    RTF_ASSERT_ERROR_IF(cmode==VOCAB_CM_POSITION_DIRECT,"Unable to set control mode: VOCAB_CM_POSITION_DIRECT");

    double start_time = yarp::os::Time::now();
    double previous_cmd=zero;
    const double max_step = 1.0;
    while(1)
    {
        double curr_time = yarp::os::Time::now();
        double elapsed = curr_time-start_time;
        double cmd = amplitude*sin(2*3.14159265359*frequency*elapsed)+zero;
        if (fabs(previous_cmd-cmd)>max_step)
        {
            char buff[255];
            sprintf(buff,"error in signal generation: previous: %+6.3f current: %+6.3f max step:  %+6.3f",previous_cmd,cmd,max_step);
            RTF_ASSERT_ERROR(buff);
        }
        idir->setPosition(0,cmd);
        previous_cmd = cmd;
        
        //printf("%+6.3f %f\n",elapsed, cmd);
        yarp::os::Time::delay(sampleTime);
        if (elapsed*frequency>cycles) break;
    }

    goHome();
}
