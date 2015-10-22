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
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include "jointLimits.h"
#include <yarp/manager/localbroker.h>

#include <stdio.h>

using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

// prepare the plugin
PREPARE_PLUGIN(JointLimits)

JointLimits::JointLimits() : YarpTestCase("JointLimits") {
    jointsList=0;
    dd=0;
    ipos=0;
    icmd=0;
    iimd=0;
    ienc=0;
    ilim=0;
    enc_jnt=0;
    original_pids=0;
    pids_saved=false;
}

JointLimits::~JointLimits() { }

bool JointLimits::setup(yarp::os::Property& property) {
    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"),       "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"),        "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"),      "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("home"),        "The home position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("speed"),       "The positionMove reference speed must be given as the test parameter!");

    RTF_ASSERT_ERROR_IF(property.check("outputLimitPercent"),  "The outputLimitPercent must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("tolerance"), "  The max error tolerance must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    char buff[500];
    sprintf(buff, "setting up test for %s", partName.c_str());
    RTF_TEST_REPORT(buff);

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* homeBottle = property.find("home").asList();
    RTF_ASSERT_ERROR_IF(homeBottle!=0,"unable to parse zero parameter");

    Bottle* speedBottle = property.find("speed").asList();
    RTF_ASSERT_ERROR_IF(speedBottle!=0,"unable to parse speed parameter");

    Bottle* outputLimitBottle = property.find("outputLimitPercent").asList();
    RTF_ASSERT_ERROR_IF(outputLimitBottle!=0,"unable to parse speed parameter");

    tolerance = property.find("tolerance").asDouble();
    RTF_ASSERT_ERROR_IF(tolerance>=0,"invalid tolerance");

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/jointsLimitsTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(ilim),"Unable to open limits interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipid),"Unable to open pid interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    enc_jnt.resize(n_part_joints);
    max_lims.resize(n_cmd_joints);
    min_lims.resize(n_cmd_joints);

    home.resize (n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    speed.resize(n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) speed[i]=speedBottle->get(i).asDouble();
    outputLimit.resize(n_cmd_joints);for (int i=0; i< n_cmd_joints; i++) outputLimit[i]=outputLimitBottle->get(i).asDouble();

    original_pids = new yarp::dev::Pid[n_cmd_joints];
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipid->getPid((int)jointsList[i],&original_pids[i]);
    }
    pids_saved=true;
    
    for (unsigned int i=0; i<jointsList.size(); i++)
    { 
        yarp::dev::Pid p = original_pids[i];
        p.max_output = p.max_output/100*outputLimit[i];
        p.max_int =    p.max_int/100*outputLimit[i];
        ipid->setPid((int)jointsList[i],p);
        yarp::os::Time::delay(0.010);
        yarp::dev::Pid t;
        ipid->getPid((int)jointsList[i],&t);

        //since pid values are double, the returned values may differ from those sent due to conversion.
        if (fabs(t.max_output-p.max_output) > 0.5  ||
            fabs(t.max_int-p.max_int) > 0.5  ||
            fabs(t.kp-p.kp) > 0.5 ||
            fabs(t.kd-p.kd) > 0.5 ||
            fabs(t.ki-p.ki) > 0.5)
        {
            RTF_ASSERT_ERROR("Unable to set output limits");
        }

    }
    return true;
}

void JointLimits::tearDown()
{
    if (original_pids)
    {
        if (pids_saved)
        {
            for (unsigned int i=0; i<jointsList.size(); i++)
            {
                ipid->setPid((int)jointsList[i],original_pids[i]);
            }
        }
        delete[] original_pids; original_pids=0;
    }
    if (dd) {delete dd; dd =0;}
}

void JointLimits::setMode(int desired_mode)
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        icmd->setControlMode((int)jointsList[i],desired_mode);
        iimd->setInteractionMode((int)jointsList[i],VOCAB_IM_STIFF);
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

void JointLimits::goTo(yarp::sig::Vector position)
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipos->setRefSpeed((int)jointsList[i],speed[i]);
        ipos->positionMove((int)jointsList[i],position[i]);
    }

    int timeout = 0;
    while (1)
    {
        int in_position=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            double tmp=0;
            ienc->getEncoder((int)jointsList[i],&tmp);
            if (fabs(tmp-position[i])<tolerance) in_position++;
        }
        if (in_position==jointsList.size()) break;
        if (timeout>100)
        {
            RTF_ASSERT_ERROR("Timeout while reaching desired position");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

bool JointLimits::goToSingle(int i, double pos, double *reached_pos)
{
    ipos->setRefSpeed((int)jointsList[i],speed[i]);
    ipos->positionMove((int)jointsList[i],pos);
    double tmp=0;

    int timeout = 0;
    while (1)
    {
        ienc->getEncoder((int)jointsList[i],&tmp);
        if (fabs(tmp-pos)<tolerance) break;

        if (timeout>100)
        {
            if(reached_pos != NULL)
            {
                *reached_pos = tmp;
            }
            return(false);
//            char buff[500];
//            sprintf(buff, "Timeout while reaching desired position. Reached pos=%.2f", tmp);
//            RTF_ASSERT_ERROR(buff);
        }

        yarp::os::Time::delay(0.2);
        timeout++;
    }
    if(reached_pos != NULL)
    {
        *reached_pos = tmp;
    }
    return(true);
}

void JointLimits::run()
{
    char buff[500];
    setMode(VOCAB_CM_POSITION);
    goTo(home);

    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ilim->getLimits((int)jointsList[i],&min_lims[i],&max_lims[i]);
        RTF_ASSERT_ERROR_IF(max_lims[i]!=min_lims[i],"Invalid limit: max==min?");
        RTF_ASSERT_ERROR_IF(max_lims[i]>min_lims[i],"Invalid limit: max<min?");
        if (max_lims[i] == 0 && min_lims[i] == 0) RTF_ASSERT_ERROR("Invalid limit: max==min==0");
    }
    
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        bool res;
        double reached_pos=0;

        //Check max limit
        sprintf(buff,"Testing joint %d, max limit: %f",(int)jointsList[i],max_lims[i]);RTF_TEST_REPORT(buff);
        res = goToSingle(i, max_lims[i], &reached_pos);
        if(!res)
        {
            goToSingle(i,home[i], NULL); //I need to go to home in order to leave joint in safe position for further tests (other body parts)
            sprintf(buff, "Timeout while reaching desired position(%.2f). Reached pos=%.2f", max_lims[i], reached_pos);RTF_ASSERT_ERROR(buff);
        }

        //Check min limit
        sprintf(buff,"Testing joint %d, min limit: %f",(int)jointsList[i],min_lims[i]);RTF_TEST_REPORT(buff);
        res = goToSingle(i, min_lims[i], &reached_pos);
        if(!res)
        {
            goToSingle(i,home[i], NULL);
            sprintf(buff, "Timeout while reaching desired position(%.2f). Reached pos=%.2f", min_lims[i], reached_pos);RTF_ASSERT_ERROR(buff);
        }

        //Check home position
        sprintf(buff,"Testing joint %d, homing to: %f",(int)jointsList[i],home[i]);RTF_TEST_REPORT(buff);
        res = goToSingle(i, home[i], &reached_pos);
        if(!res)
        {
            sprintf(buff, "Timeout while reaching desired position(%.2f). Reached pos=%.2f", home[i], reached_pos);RTF_ASSERT_ERROR(buff);
        }

    }
    ienc->getEncoders(enc_jnt.data());

    goTo(home);
    yarp::os::Time::delay(2.0);
}
