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
#include <yarp/os/Property.h>

#include "ControlModes.h"

//example1    -v -t ControlModes.dll -p "--robot icub --part head --joints ""(0)"" --zero 0"
//example2    -v -t ControlModes.dll -p "--robot icub --part head --joints ""(0 1 2)"" --zero 0 "
//example3    -v -t ControlModes.dll -p "--robot icub --part head --joints ""(0 1 2 3 4 5)"" --zero 0"

using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;

// prepare the plugin
PREPARE_PLUGIN(ControlModes)

ControlModes::ControlModes() : YarpTestCase("ControlModes") {
    jointsList=0;
    pos_tot=0;
    dd=0;
    ipos=0;
    iamp=0;
    icmd=0;
    iimd=0;
    ienc=0;
    idir=0;
    itrq=0;
    ivel=0;
    cmd_some=0;
    cmd_tot=0;
    prevcurr_some=0;
    prevcurr_tot=0;
}

ControlModes::~ControlModes() { }

bool ControlModes::setup(yarp::os::Property& property) {

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"), "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"), "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("zero"),    "The zero position must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    zero = property.find("zero").asDouble();

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");
    n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0,"invalid number of joints, it must be >0");

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/ControlModesTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(idir),"Unable to open position direct interface");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(iamp),"Unable to open ampliefier interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(ivel),"Unable to open velocity control interface");
    RTF_ASSERT_ERROR_IF(dd->view(itrq),"Unable to open torque control interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    if      (n_part_joints<=0)
        RTF_ASSERT_ERROR("Error this part has in invalid (<=0) number of jonits");
    else if (jointsBottle->size() == 1)
        cmd_mode=single_joint;
    else if (jointsBottle->size() < n_part_joints)
        cmd_mode=some_joints;
    else if (jointsBottle->size() == n_part_joints)
        cmd_mode=all_joints;
    else
        RTF_ASSERT_ERROR("invalid joint selection?");

    cmd_tot = new double[n_part_joints];
    pos_tot=new double[n_part_joints];
    jointsList=new int[n_cmd_joints];
    cmd_some=new double[n_cmd_joints];
    prevcurr_tot=new double[n_part_joints];
    prevcurr_some=new double[n_cmd_joints];
    for (int i=0; i <n_cmd_joints; i++) jointsList[i]=jointsBottle->get(i).asInt();

    return true;
}

void ControlModes::tearDown()
{
    if (jointsList) {delete jointsList; jointsList =0;}
    if (dd) {delete dd; dd =0;}
}

void ControlModes::getOriginalCurrentLimits()
{
    for (int i=0; i<n_cmd_joints; i++)
    {
        iamp->getMaxCurrent(jointsList[i],&prevcurr_some[i]);
    }
    for (int i=0; i<n_part_joints; i++)
    {
        iamp->getMaxCurrent(i,&prevcurr_tot[i]);
    }
    yarp::os::Time::delay(0.010);
}

void ControlModes::resetOriginalCurrentLimits()
{
    for (int i=0; i<n_part_joints; i++)
    {
        iamp->setMaxCurrent(i,prevcurr_tot[i]);
    }
    yarp::os::Time::delay(0.010);
}

void ControlModes::zeroCurrentLimits()
{
    if (cmd_mode==single_joint)
    {
        for (int i=0; i<n_cmd_joints; i++)
        {
            iamp->setMaxCurrent(jointsList[i],0.0);
        }
    }
    else if (cmd_mode==some_joints)
    {
        for (int i=0; i<n_cmd_joints; i++)
        {
            iamp->setMaxCurrent(jointsList[i],0.0);
        }
    }
    else if (cmd_mode==all_joints)
    {
        for (int i=0; i<n_part_joints; i++)
        {
            iamp->setMaxCurrent(i,0.0);
        }
    }
    else
    {
        RTF_ASSERT_ERROR("Invalid cmd_mode");
    }
    yarp::os::Time::delay(0.010);
}

void ControlModes::setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode)
{
    for (int i=0; i<n_cmd_joints; i++)
    {
        icmd->setControlMode(jointsList[i],desired_control_mode);
        iimd->setInteractionMode(jointsList[i],desired_interaction_mode);
        yarp::os::Time::delay(0.010);
    }
}

void ControlModes::verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, yarp::os::ConstString title)
{
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
        for (int i=0; i<n_cmd_joints; i++)
        {
            icmd->getControlMode (jointsList[i],&cmode);
            iimd->getInteractionMode(jointsList[i],&imode);
            if (cmode==desired_control_mode && imode==desired_interaction_mode) ok++;
        }
        if (ok==n_cmd_joints) break;
        if (timeout>100)
        {
            char sbuf[500];
            sprintf(sbuf,"Test (%s) failed: current mode is (%d,%d), it should be (%d,%d)",title.c_str(), cmode,imode,desired_control_mode,desired_interaction_mode);
            RTF_ASSERT_ERROR(sbuf);
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
    char sbuf[500];
    sprintf(sbuf,"Test (%s) passed: current mode is (%d,%d)",title.c_str(), desired_control_mode,desired_interaction_mode);
    RTF_TEST_REPORT(sbuf);
}

void ControlModes::verifyAmplifier(int desired_amplifier_mode, yarp::os::ConstString title)
{
    int amode;
    int timeout = 0;

    while (1)
    {
        int ok=0;
        for (int i=0; i<n_cmd_joints; i++)
        {
            iamp->getAmpStatus   (jointsList[i],&amode);
            //@@@@@@
            amode = 0; // to complete using the proper interface
            //@@@@@@
            if (amode==desired_amplifier_mode) ok++;
        }
        if (ok==n_cmd_joints) break;
        if (timeout>100)
        {
            char sbuf[500];
            sprintf(sbuf,"Test (%s) failed: amplifier mode is (%d), it should be (%d)",title.c_str(), amode, desired_amplifier_mode);
            RTF_ASSERT_ERROR(sbuf);
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
    char sbuf[500];
    sprintf(sbuf,"Test (%s) passed: amplifier mode is (%d)",title.c_str(), desired_amplifier_mode);
    RTF_TEST_REPORT(sbuf);
}

void ControlModes::executeCmd()
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
}

void ControlModes::goHome()
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

void ControlModes::run()
{
    getOriginalCurrentLimits();
    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test0");
    verifyAmplifier(0,"test0b"); //@@@@@@ To be completed
    goHome();

    //------ check all modes when stiff ------
    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test1");
    verifyAmplifier(0,"test1b");

    setMode(VOCAB_CM_POSITION_DIRECT,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION_DIRECT,VOCAB_IM_STIFF,"test2");
    verifyAmplifier(0,"test2b");

    setMode(VOCAB_CM_VELOCITY,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_VELOCITY,VOCAB_IM_STIFF,"test3");
    verifyAmplifier(0,"test3b");

    setMode(VOCAB_CM_TORQUE,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_TORQUE,VOCAB_IM_STIFF,"test4");
    verifyAmplifier(0,"test4b");
<<<<<<< HEAD

=======
    
>>>>>>> origin/master
    setMode(VOCAB_CM_MIXED,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_MIXED,VOCAB_IM_STIFF,"test5");
    verifyAmplifier(0,"test5b");

    setMode(VOCAB_CM_OPENLOOP,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_OPENLOOP,VOCAB_IM_STIFF,"test6");
    verifyAmplifier(0,"test6b");

    setMode(VOCAB_CM_IDLE,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_IDLE,VOCAB_IM_STIFF,"test7");
    verifyAmplifier(0,"test7b");

    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test8");
    verifyAmplifier(0,"test8b");

    setMode(VOCAB_CM_FORCE_IDLE,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_IDLE,VOCAB_IM_STIFF,"test9"); //VOCAB_CM_IDLE is intentional
    verifyAmplifier(0,"test9b");

    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test10");
    verifyAmplifier(0,"test10b");
    goHome();

    //------ check all modes when compliant ------
    setMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT,"test11");
    verifyAmplifier(0,"test11b");

    setMode(VOCAB_CM_POSITION_DIRECT,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_POSITION_DIRECT,VOCAB_IM_COMPLIANT,"test12");
    verifyAmplifier(0,"test12b");

    setMode(VOCAB_CM_VELOCITY,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_VELOCITY,VOCAB_IM_COMPLIANT,"test13");
    verifyAmplifier(0,"test13b");

    setMode(VOCAB_CM_TORQUE,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_TORQUE,VOCAB_IM_COMPLIANT,"test14");
    verifyAmplifier(0,"test14b");

    setMode(VOCAB_CM_MIXED,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_MIXED,VOCAB_IM_COMPLIANT,"test15");
    verifyAmplifier(0,"test15b");
<<<<<<< HEAD

=======
    
>>>>>>> origin/master
    setMode(VOCAB_CM_OPENLOOP,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_OPENLOOP,VOCAB_IM_COMPLIANT,"test16");
    verifyAmplifier(0,"test16b");

    setMode(VOCAB_CM_IDLE,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_IDLE,VOCAB_IM_COMPLIANT,"test17");
    verifyAmplifier(0,"test17b");

    setMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT,"test18");
    verifyAmplifier(0,"test18b");

    setMode(VOCAB_CM_FORCE_IDLE,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_IDLE,VOCAB_IM_COMPLIANT,"test19"); //VOCAB_CM_IDLE is intentional
    verifyAmplifier(0,"test19b");

    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test20");
    verifyAmplifier(0,"test20b");
    goHome();

    //------ create an intentional HW FAULT ------
    zeroCurrentLimits();
    verifyMode(VOCAB_CM_HW_FAULT, VOCAB_IM_STIFF,"test21");
    verifyAmplifier(0,"test21b");
    resetOriginalCurrentLimits();

    //------ check all modes when in HW_FAULT ------
    setMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_HW_FAULT,VOCAB_IM_COMPLIANT,"test22");
    verifyAmplifier(0,"test22b");

    setMode(VOCAB_CM_POSITION_DIRECT,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_HW_FAULT,VOCAB_IM_COMPLIANT,"test23");
    verifyAmplifier(0,"test23b");

    setMode(VOCAB_CM_VELOCITY,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_HW_FAULT,VOCAB_IM_COMPLIANT,"test24");
    verifyAmplifier(0,"test24b");

    setMode(VOCAB_CM_TORQUE,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_HW_FAULT,VOCAB_IM_COMPLIANT,"test25");
    verifyAmplifier(0,"test25b");

    setMode(VOCAB_CM_MIXED,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_HW_FAULT,VOCAB_IM_COMPLIANT,"test26");
    verifyAmplifier(0,"test26b");
<<<<<<< HEAD

=======
    
>>>>>>> origin/master
    setMode(VOCAB_CM_OPENLOOP,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_HW_FAULT,VOCAB_IM_COMPLIANT,"test27");
    verifyAmplifier(0,"test27b");

    setMode(VOCAB_CM_IDLE,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_HW_FAULT,VOCAB_IM_COMPLIANT,"test28");
    verifyAmplifier(0,"test28b");

    setMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_HW_FAULT,VOCAB_IM_COMPLIANT,"test29");
    verifyAmplifier(0,"test29b");

    setMode(VOCAB_CM_FORCE_IDLE,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_IDLE,VOCAB_IM_COMPLIANT,"test30"); //VOCAB_CM_IDLE is intentional
    verifyAmplifier(0,"test30b");

    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test31");
    verifyAmplifier(0,"test31b");
    goHome();
}
