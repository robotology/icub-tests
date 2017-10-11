// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <math.h>
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include "PositionDirect.h"

using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;

// prepare the plugin
PREPARE_PLUGIN(PositionDirect)

PositionDirect::PositionDirect() : yarp::rtf::TestCase("PositionDirect") {
    jointsList=0;
    pos_tot=0;
    dd=0;
    ipos=0;
    icmd=0;
    iimd=0;
    ienc=0;
    idir=0;
    cmd_some=0;
    cmd_tot=0;
}

PositionDirect::~PositionDirect() { }

bool PositionDirect::setup(yarp::os::Property& property) {

    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    RTF_ASSERT_ERROR_IF_FALSE(property.check("robot"), "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("part"), "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("zero"),    "The zero position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("frequency"), "The frequency of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("amplitude"), "The amplitude of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("cycles"), "The number of cycles of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("tolerance"), "The tolerance of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("sampleTime"), "The sampleTime of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("cmdMode"), "the cmdType must be given as the test parameter! 0=single_joint, 1=all_joints, 2=some_joints");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF_FALSE(jointsBottle!=0,"unable to parse joints parameter");
    n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF_FALSE(n_cmd_joints>0,"invalid number of joints, it must be >0");

    frequency = property.find("frequency").asDouble();
    RTF_ASSERT_ERROR_IF_FALSE(frequency>0,"invalid frequency");

    amplitude = property.find("amplitude").asDouble();
    RTF_ASSERT_ERROR_IF_FALSE(amplitude>0,"invalid amplitude");

    zero = property.find("zero").asDouble();

    cycles = property.find("cycles").asDouble();
    RTF_ASSERT_ERROR_IF_FALSE(cycles>0,"invalid cycles");

    tolerance = property.find("tolerance").asDouble();
    RTF_ASSERT_ERROR_IF_FALSE(tolerance>0,"invalid tolerance");

    sampleTime = property.find("sampleTime").asDouble();
    RTF_ASSERT_ERROR_IF_FALSE(sampleTime>0,"invalid sampleTime");

    cmd_mode = (cmd_mode_t) property.find("cmdMode").asInt();
    RTF_ASSERT_ERROR_IF_FALSE(cmd_mode>=0 && cmd_mode<=2,"invalid cmdMode: can be 0=single_joint, 1=all_joints ,2=some_joints");

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/positionDirectTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF_FALSE(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(idir),"Unable to open position direct interface");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(iimd),"Unable to open interaction mode interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    if (cmd_mode==all_joints && n_part_joints!=n_cmd_joints)
    {
        RTF_ASSERT_ERROR("if all_joints=2 mode is selected, joints parameter must include the full list of joints");
    }

    if (cmd_mode==single_joint && n_cmd_joints!=1)
    {
        RTF_ASSERT_ERROR("if single_joint=1 mode is selected, joints parameter must include one single joint");
    }

    cmd_tot = new double[n_part_joints];
    pos_tot=new double[n_part_joints];
    jointsList=new int[n_cmd_joints];
    cmd_some=new double[n_cmd_joints];
    for (int i=0; i <n_cmd_joints; i++) jointsList[i]=jointsBottle->get(i).asInt();

    return true;
}

void PositionDirect::tearDown()
{
    if (jointsList) {delete jointsList; jointsList =0;}
    if (dd) {delete dd; dd =0;}
}

void PositionDirect::setMode(int desired_mode)
{
    for (int i=0; i<n_cmd_joints; i++)
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
        for (int i=0; i<n_cmd_joints; i++)
        {
            icmd->getControlMode (jointsList[i],&cmode);
            iimd->getInteractionMode(jointsList[i],&imode);
            if (cmode==desired_mode && imode==VOCAB_IM_STIFF) ok++;
        }
        if (ok==n_cmd_joints) break;
        if (timeout>100)
        {
            RTF_ASSERT_ERROR("Unable to set control mode/interaction mode");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

void PositionDirect::executeCmd()
{
    if (cmd_mode==single_joint)
    {
        for (int i=0; i<n_cmd_joints; i++)
        {
            idir->setPosition(jointsList[i],cmd_single);
        }
    }
    else if (cmd_mode==some_joints)
    {
        for (int i=0; i<n_cmd_joints; i++)
        {
            cmd_some[i]=cmd_single;
        }
        idir->setPositions(n_cmd_joints,jointsList, cmd_some);
    }
    else if (cmd_mode==all_joints)
    {
        for (int i=0; i<n_part_joints; i++)
        {
            cmd_tot[i]=cmd_single;
        }
        idir->setPositions(cmd_tot);
    }
    else
    {
        RTF_ASSERT_ERROR("Invalid cmd_mode");
    }

    prev_cmd=cmd_single;
}

void PositionDirect::goHome()
{
    for (int i=0; i<n_cmd_joints; i++)
    {
        ipos->setRefSpeed(jointsList[i],20.0);
        ipos->positionMove(jointsList[i],zero);
    }

    int timeout = 0;
    while (1)
    {
        int in_position=0;
        for (int i=0; i<n_cmd_joints; i++)
        {
            ienc->getEncoder(jointsList[i],&pos_tot[jointsList[i]]);
            if (fabs(pos_tot[jointsList[i]]-zero)<0.5) in_position++;
        }
        if (in_position==n_cmd_joints) break;
        if (timeout>100)
        {
            RTF_ASSERT_ERROR("Timeout while reaching zero position");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

void PositionDirect::run()
{
    setMode(VOCAB_CM_POSITION);
    goHome();
    setMode(VOCAB_CM_POSITION_DIRECT);

    double start_time = yarp::os::Time::now();
    const double max_step = 2.0;
    prev_cmd=cmd_single = amplitude*sin(0.0)+zero;
    while(1)
    {
        double curr_time = yarp::os::Time::now();
        double elapsed = curr_time-start_time;
        cmd_single = amplitude*(2*3.14159265359*frequency*elapsed)+zero;

        RTF_ASSERT_ERROR_IF_FALSE(fabs(prev_cmd-cmd_single)<max_step,
                            Asserter::format("error in signal generation: previous: %+6.3f current: %+6.3f max step:  %+6.3f",
                                             prev_cmd,cmd_single,max_step));
        ienc->getEncoders(pos_tot);
        executeCmd();
        //printf("%+6.3f %f\n",elapsed, cmd);
        yarp::os::Time::delay(sampleTime);
        if (elapsed*frequency>cycles) break;
    }

    setMode(VOCAB_CM_POSITION);
    goHome();
}
