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

#include "OpenloopConsistency.h"

//example1    -v -t OpenLoopConsistency.dll -p "--robot icub --part head --joints ""(0)"" --home ""(0)"" "
//example2    -v -t OpenLoopConsistency.dll -p "--robot icub --part head --joints ""(0 1 2)"" --home ""(0 0 0)"" "
//example3    -v -t OpenLoopConsistency.dll -p "--robot icub --part head --joints ""(0 1 2 3 4 5)"" --home ""(0 0 0 0 0 10)"" "

using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;

// prepare the plugin
PREPARE_PLUGIN(OpenLoopConsistency)

OpenLoopConsistency::OpenLoopConsistency() : yarp::rtf::TestCase("OpenLoopConsistency") {
    jointsList=0;
    pos_tot=0;
    dd=0;
    ipos=0;
    iamp=0;
    icmd=0;
    iimd=0;
    ienc=0;
    ipwm = 0;
    cmd_some=0;
    cmd_tot=0;
    prevcurr_some=0;
    prevcurr_tot=0;
}

OpenLoopConsistency::~OpenLoopConsistency() { }

bool OpenLoopConsistency::setup(yarp::os::Property& property) {

    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"), "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"), "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("home"),    "The home positions must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* homeBottle = property.find("home").asList();
    RTF_ASSERT_ERROR_IF(homeBottle!=0,"unable to parse home parameter");

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");
    n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0,"invalid number of joints, it must be >0");

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/OpenLoopConsistencyTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(ipwm), "Unable to open pwm control interface");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(iamp),"Unable to open ampliefier interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");

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
    home=new double[n_cmd_joints];
    for (int i=0; i <n_cmd_joints; i++) jointsList[i]=jointsBottle->get(i).asInt();
    for (int i=0; i <n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    return true;
}

void OpenLoopConsistency::tearDown()
{

    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    goHome();


    if (jointsList) {delete jointsList; jointsList =0;}
    if(home){delete [] home; home=0;}
    if (dd) {delete dd; dd =0;}
}

void OpenLoopConsistency::setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode)
{
    for (int i=0; i<n_cmd_joints; i++)
    {
        icmd->setControlMode(jointsList[i],desired_control_mode);
        iimd->setInteractionMode(jointsList[i],desired_interaction_mode);
        yarp::os::Time::delay(0.010);
    }
}

void OpenLoopConsistency::verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, yarp::os::ConstString title)
{
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
            if (cmode==desired_control_mode && imode==desired_interaction_mode) ok++;
        }
        if (ok==n_cmd_joints) break;
        if (timeout>100)
        {
            char sbuf[500];
            sprintf(sbuf,"Test (%s) failed: current mode is (%s,%s), it should be (%s,%s)",title.c_str(), Vocab::decode((NetInt32)desired_control_mode).c_str(),Vocab::decode((NetInt32)desired_interaction_mode).c_str(), Vocab::decode((NetInt32)cmode).c_str(),Vocab::decode((NetInt32)imode).c_str());
            RTF_ASSERT_ERROR(sbuf);
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
    char sbuf[500];
    sprintf(sbuf,"Test (%s) passed: current mode is (%s,%s)",title.c_str(), Vocab::decode((NetInt32)desired_control_mode).c_str(),Vocab::decode((NetInt32)desired_interaction_mode).c_str());
    RTF_TEST_REPORT(sbuf);
}

void OpenLoopConsistency::goHome()
{
    for (int i=0; i<n_cmd_joints; i++)
    {
        ipos->setRefSpeed(jointsList[i],20.0);
        ipos->positionMove(jointsList[i],home[i]);
    }

    int timeout = 0;
    while (1)
    {
        int in_position=0;
        for (int i=0; i<n_cmd_joints; i++)
        {
            ienc->getEncoder(jointsList[i],&pos_tot[jointsList[i]]);
            if (fabs(pos_tot[jointsList[i]]-home[i])<0.5) in_position++;
        }
        if (in_position==n_cmd_joints) break;
        if (timeout>100)
        {
            RTF_ASSERT_ERROR("Timeout while reaching home position");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

void OpenLoopConsistency::setRefOpenloop(double value)
{
    cmd_single=value;
    if (cmd_mode==single_joint)
    {
        for (int i=0; i<n_cmd_joints; i++)
        {
            ipwm->setRefDutyCycle(jointsList[i], cmd_single);
        }
    }
    else if (cmd_mode==some_joints)
    {
        //same of single_joint, since multiple joint is not currently supported
        for (int i=0; i<n_cmd_joints; i++)
        {
            ipwm->setRefDutyCycle(jointsList[i], cmd_single);
        }
    }
    else if (cmd_mode==all_joints)
    {
        for (int i=0; i<n_part_joints; i++)
        {
            cmd_tot[i]=cmd_single;
        }
        ipwm->setRefDutyCycles(cmd_tot);
    }
    else
    {
        RTF_ASSERT_ERROR("Invalid cmd_mode");
    }
    yarp::os::Time::delay(0.010);
}

void OpenLoopConsistency::verifyRefOpenloop(double verify_val, yarp::os::ConstString title)
{
    double value;
    char sbuf[500];
    if (cmd_mode==single_joint)
    {
        for (int i=0; i<n_cmd_joints; i++)
        {
            ipwm->getRefDutyCycle(jointsList[i], &value);
            if (value==verify_val)
            {
                sprintf(sbuf,"Test (%s) passed, j%d current reference is (%f)",title.c_str(),i, verify_val);
                RTF_TEST_REPORT(sbuf);
            }
            else
            {
                sprintf(sbuf,"Test (%s) failed: current reference is (%f), it should be (%f)",title.c_str(), value, verify_val);
                RTF_ASSERT_ERROR(sbuf);
            }
        }
    }
    else if (cmd_mode==some_joints)
    {
        //same of single_joint, since multiple joint is not currently supported
        for (int i=0; i<n_cmd_joints; i++)
        {
            ipwm->getRefDutyCycle(jointsList[i], &value);
            if (value==verify_val)
            {
                sprintf(sbuf,"Test (%s) passed j%d current reference is (%f)",title.c_str(),i, verify_val);
                RTF_TEST_REPORT(sbuf);
            }
            else
            {
                sprintf(sbuf,"Test (%s) failed: current reference is (%f), it should be (%f)",title.c_str(), value, verify_val);
                RTF_ASSERT_ERROR(sbuf);
            }
        }
    }
    else if (cmd_mode==all_joints)
    {
        int ok=0;
        ipwm->getRefDutyCycles(cmd_tot);
        for (int i=0; i<n_part_joints; i++)
        {
            if (verify_val==cmd_tot[i]) ok++;
        }
        if (ok==n_part_joints)
        {
            sprintf(sbuf,"Test (%s) passed, current reference is (%f)",title.c_str(), verify_val);
            RTF_TEST_REPORT(sbuf);
        }
        else
        {
            sprintf(sbuf,"Test (%s) failed: only %d joints (of %d) are ok",title.c_str(),ok,n_part_joints);
            RTF_ASSERT_ERROR(sbuf);
        }
    }
    else
    {
        RTF_ASSERT_ERROR("Invalid cmd_mode");
    }
    yarp::os::Time::delay(0.010);
}

void OpenLoopConsistency::verifyOutputEqual(double verify_val, yarp::os::ConstString title)
{
    double value;
    char sbuf[500];
    if (cmd_mode==single_joint)
    {
        for (int i=0; i<n_cmd_joints; i++)
        {
            ipwm->getDutyCycle(jointsList[i], &value);
            if (value==verify_val)
            {
                sprintf(sbuf,"Test (%s) passed, j%d current output is (%f)",title.c_str(),i, verify_val);
                RTF_TEST_REPORT(sbuf);
            }
            else
            {
                sprintf(sbuf,"Test (%s) failed: current output is (%f), it should be (%f)",title.c_str(), value, verify_val);
                RTF_ASSERT_ERROR(sbuf);
            }
        }
    }
    else if (cmd_mode==some_joints)
    {
        //same of single_joint, since multiple joint is not currently supported
        for (int i=0; i<n_cmd_joints; i++)
        {
            ipwm->getDutyCycle(jointsList[i], &value);
            if (value==verify_val)
            {
                sprintf(sbuf,"Test (%s) passed j%d current output is (%f)",title.c_str(),i,verify_val);
                RTF_TEST_REPORT(sbuf);
            }
            else
            {
                sprintf(sbuf,"Test (%s) failed: current output is (%f), it should be (%f)",title.c_str(), value, verify_val);
                RTF_ASSERT_ERROR(sbuf);
            }
        }
    }
    else if (cmd_mode==all_joints)
    {
        int ok=0;
        ipwm->getDutyCycles(cmd_tot);
        for (int i=0; i<n_part_joints; i++)
        {
            if (verify_val==cmd_tot[i]) ok++;
            else
            {
                RTF_TEST_REPORT(RTF::Asserter::format("verify_val=%.2f, read_val=%.2f j=%d",verify_val, cmd_tot[i], i ));
            }
        }
        if (ok==n_part_joints)
        {
            sprintf(sbuf,"Test (%s) passed current output is (%f)",title.c_str(), verify_val);
            RTF_TEST_REPORT(sbuf);
        }
        else
        {
            sprintf(sbuf,"Test (%s) failed: only %d joints (of %d) are ok",title.c_str(),ok,n_part_joints);
            RTF_ASSERT_ERROR(sbuf);
        }
    }
    else
    {
        RTF_ASSERT_ERROR("Invalid cmd_mode");
    }
    yarp::os::Time::delay(0.010);
}

void OpenLoopConsistency::verifyOutputDiff(double verify_val, yarp::os::ConstString title)
{
    double value;
    char sbuf[500];
    if (cmd_mode==single_joint)
    {
        for (int i=0; i<n_cmd_joints; i++)
        {
            ipwm->getDutyCycle(jointsList[i], &value);
            if (value!=verify_val)
            {
                sprintf(sbuf,"Test (%s) passed j%d, current output is (%f!=%f)",title.c_str(),i,value,verify_val);
                RTF_TEST_REPORT(sbuf);
            }
            else
            {
                sprintf(sbuf,"Test (%s) failed: current output is (%f), it should be (%f)",title.c_str(), value, verify_val);
                RTF_ASSERT_ERROR(sbuf);
            }
        }
    }
    else if (cmd_mode==some_joints)
    {
        //same of single_joint, since multiple joint is not currently supported
        for (int i=0; i<n_cmd_joints; i++)
        {
            ipwm->getDutyCycle(jointsList[i], &value);
            if (value!=verify_val)
            {
                sprintf(sbuf,"Test (%s) passed j%d current output is (%f!=%f)",title.c_str(), i,value,verify_val);
                RTF_TEST_REPORT(sbuf);
            }
            else
            {
                sprintf(sbuf,"Test (%s) failed: current output is (%f), it should be (%f)",title.c_str(), value, verify_val);
                RTF_ASSERT_ERROR(sbuf);
            }
        }
    }
    else if (cmd_mode==all_joints)
    {
        int ok=0;
        ipwm->getDutyCycles(cmd_tot);
        for (int i=0; i<n_part_joints; i++)
        {
            if (verify_val!=cmd_tot[i]) ok++;
        }
        if (ok==n_part_joints)
        {
            sprintf(sbuf,"Test (%s) passed current output is (%f!=%f)",title.c_str(),value,verify_val);
            RTF_TEST_REPORT(sbuf);
        }
        else
        {
            sprintf(sbuf,"Test (%s) failed: only %d joints (of %d) are ok",title.c_str(),ok,n_part_joints);
            RTF_ASSERT_ERROR(sbuf);
        }
    }
    else
    {
        RTF_ASSERT_ERROR("Invalid cmd_mode");
    }
    yarp::os::Time::delay(0.010);
}

void OpenLoopConsistency::run()
{
    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test0");
    goHome();
    //verifyRefOpenloop(0,"test0a"); if I get openLoop reference I get last given refrence
    //verifyOutputDiff(0,"test0b"); //TO BE CHECKED

    setMode(VOCAB_CM_PWM,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_PWM, VOCAB_IM_STIFF, "test1");
    verifyRefOpenloop(0,"test0a"); //When joint swap in pwm control mode, the reference should be 0.
    verifyOutputEqual(0,"test1a");// here i can check that iPwm reference = 0.
    setRefOpenloop(1);
    verifyMode(VOCAB_CM_PWM, VOCAB_IM_STIFF, "test2");
    verifyRefOpenloop(1,"test2a");
    verifyOutputEqual(1,"test2b");
    verifyMode(VOCAB_CM_PWM, VOCAB_IM_STIFF, "test3");
    setRefOpenloop(0);
    verifyMode(VOCAB_CM_PWM, VOCAB_IM_STIFF, "test4");
    verifyRefOpenloop(0,"test3a");
    verifyOutputEqual(0,"test3b");
    verifyMode(VOCAB_CM_PWM, VOCAB_IM_STIFF, "test5");
    setRefOpenloop(-1);
    verifyMode(VOCAB_CM_PWM, VOCAB_IM_STIFF, "test6");
    verifyRefOpenloop(-1,"test6a");
    verifyOutputEqual(-1,"test6b");

    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test7");
    goHome();
    verifyRefOpenloop(-1,"test7a");
    //verifyOutputDiff(0,"test7b"); //TO BE CHECKED

}
