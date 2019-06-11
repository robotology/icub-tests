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

#include <math.h>
#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include "jointLimits.h"

#include <stdio.h>

using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(JointLimits)

JointLimits::JointLimits() : yarp::robottestingframework::TestCase("JointLimits") {
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
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("robot"),       "The robot name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("part"),        "The part name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("joints"),      "The joints list must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("home"),        "The home position must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("speed"),       "The positionMove reference speed must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("outOfBoundPosition"), "The outOfBoundPosition parameter must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("outputLimitPercent"),  "The outputLimitPercent must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("tolerance"), "  The max error tolerance must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    char buff[500];
    sprintf(buff, "setting up test for %s", partName.c_str());
    ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);

    Bottle* jointsBottle = property.find("joints").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* homeBottle = property.find("home").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(homeBottle!=0,"unable to parse zero parameter");

    Bottle* speedBottle = property.find("speed").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(speedBottle!=0,"unable to parse speed parameter");

    Bottle* outOfBoundPosition = property.find("outOfBoundPosition").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(outOfBoundPosition != 0, "unable to parse outOfBoundPosition parameter");

    Bottle* outputLimitBottle = property.find("outputLimitPercent").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(outputLimitBottle!=0,"unable to parse speed parameter");

    tolerance = property.find("tolerance").asDouble();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(tolerance>=0,"invalid tolerance");

    Bottle* toleranceListBottle = property.find("toleranceList").asList(); //optional param

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/jointsLimitsTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->isValid(),"Unable to open device driver");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ienc),"Unable to open encoders interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ipos),"Unable to open position interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(icmd),"Unable to open control mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iimd),"Unable to open interaction mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ilim),"Unable to open limits interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ipid),"Unable to open pid interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    enc_jnt.resize(n_part_joints);
    max_lims.resize(n_cmd_joints);
    min_lims.resize(n_cmd_joints);

    home.resize (n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    speed.resize(n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) speed[i]=speedBottle->get(i).asDouble();
    outputLimit.resize(n_cmd_joints);for (int i=0; i< n_cmd_joints; i++) outputLimit[i]=outputLimitBottle->get(i).asDouble();
    outOfBoundPos.resize(n_cmd_joints); for (int i = 0; i < n_cmd_joints; i++) { outOfBoundPos[i] = outOfBoundPosition->get(i).asDouble(); ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(outOfBoundPos[i] > 0 , "outOfBoundPosition must be > 0"); }
    toleranceList.resize(n_cmd_joints);
    for (int i = 0; i < n_cmd_joints; i++)
    {
        if(toleranceListBottle)
        {
            toleranceList[i] = toleranceListBottle->get(i).asDouble();
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(toleranceList[i] >= 0 , "toleranceList must be > 0");
        }
        else
            toleranceList[i] = tolerance;
    }

    original_pids = new yarp::dev::Pid[n_cmd_joints];
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipid->getPid(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, (int)jointsList[i],&original_pids[i]);
    }
    pids_saved=true;

    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        yarp::dev::Pid p = original_pids[i];
        p.max_output = p.max_output/100*outputLimit[i];
        p.max_int =    p.max_int/100*outputLimit[i];
        ipid->setPid(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, (int)jointsList[i],p);
        yarp::os::Time::delay(0.010);
        yarp::dev::Pid t;
        ipid->getPid(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, (int)jointsList[i],&t);

        //since pid values are double, the returned values may differ from those sent due to conversion.
        if (fabs(t.max_output-p.max_output) > 1.0  ||
            fabs(t.max_int-p.max_int) > 1.0  ||
            fabs(t.kp-p.kp) > 1.0 ||
            fabs(t.kd-p.kd) > 1.0 ||
            fabs(t.ki-p.ki) > 1.0)
        {
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("Unable to set output limits");
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
                ipid->setPid(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, (int)jointsList[i],original_pids[i]);
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
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("Unable to set control mode/interaction mode");
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
            if (fabs(tmp-position[i])<toleranceList[i]) in_position++;
        }
        if (in_position==jointsList.size()) break;
        if (timeout>100)
        {
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("Timeout while reaching desired position");
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
        if (fabs(tmp-pos)<toleranceList[i]) break;

        if (timeout>100)
        {
            if(reached_pos != NULL)
            {
                *reached_pos = tmp;
            }
            return(false);
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

bool JointLimits::goToSingleExceed(int i, double position_to_reach, double limit, double reachedLimit, double *reached_pos)
{
    ipos->setRefSpeed((int)jointsList[i], speed[i]);
    ipos->positionMove((int)jointsList[i], position_to_reach);
    double tmp = 0;
    bool excededLimit = false;
    int timeout = 0;
    double limitToCheck;
    if(fabs(reachedLimit-limit)>toleranceList[i])
    {
        //if i'm here joint did NOT reached the limit and I would check joint doesn't exced reached limit
        limitToCheck = reachedLimit;
    }
    else
    {
        limitToCheck = limit;
    }
    while (1)
    {
        ienc->getEncoder((int)jointsList[i], &tmp);
        if(fabs(tmp - limitToCheck)>toleranceList[i])
        {
            excededLimit = true;
            //ROBOTTESTINGFRAMEWORK_TEST_REPORT (Asserter::format("j %d exced limitTocheck %f: reached %f",(int)jointsList[i],  limitToCheck, tmp));
            break;
        }
        if (fabs(tmp - position_to_reach)<toleranceList[i])
        {
            excededLimit = true;
            //ROBOTTESTINGFRAMEWORK_TEST_REPORT (Asserter::format("j %d is near position_to_reach %f: reached %f",(int)jointsList[i],  position_to_reach, tmp));
            break;
        }

        if (timeout>50) break;
        yarp::os::Time::delay(0.2);
        timeout++;
    }

    *reached_pos = tmp;

/*    if (fabs(tmp - position_to_reach)<toleranceList[i])
    {
        //I reached the out of bound target. That's bad!
        return (true);
    }
    if (fabs(tmp - limit)<toleranceList[i])
    {
        //I'm still near the joint limit. That's fine.
        return (false);
    }
    //I dont know where I am, that's bad!
    return(true);
*/

    if( excededLimit)
        return (true);
    else
        return(false);
}


void JointLimits::run()
{
    char buff[500];
    setMode(VOCAB_CM_POSITION);
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("all joints are going to home....");
    goTo(home);

    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ilim->getLimits((int)jointsList[i],&min_lims[i],&max_lims[i]);
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(max_lims[i]!=min_lims[i],"Invalid limit: max==min?");
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(max_lims[i]>min_lims[i],"Invalid limit: max<min?");
        if (max_lims[i] == 0 && min_lims[i] == 0) ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("Invalid limit: max==min==0");
    }

    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        bool res;
        double reached_pos=0;
        double excededpos_reached = 0;

    //1) Check max limit
        sprintf(buff,"Testing if max limit is reachable, joint %d, max limit: %f",(int)jointsList[i],max_lims[i]);ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);
        //check that max_limit is reachable
        res = goToSingle(i, max_lims[i], &reached_pos);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK (res, Asserter::format("joint %d moved to max limit: %f reached: %f",  (int)jointsList[i], max_lims[i], reached_pos));
        //if(!res)
        //{
        //    goToSingle(i,home[i], NULL); //I need to go to home in order to leave joint in safe position for further tests (other body parts)
        //    sprintf(buff, "Timeout while reaching desired position(%.2f). Reached pos=%.2f", max_lims[i], reached_pos);ROBOTTESTINGFRAMEWORK_ASSERT_ERROR(buff);
        //}
        //else { sprintf(buff, "Test successfull, joint %d, max limit: %f reached: %f", (int)jointsList[i], max_lims[i], reached_pos); ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff); }

    //2) check that max_limit + outOfBoundPos is NOT reachable
        sprintf(buff, "Testing that max limit cannot be exceeded, joint %d, max limit: %f", (int)jointsList[i], max_lims[i]); ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);
        res = goToSingleExceed(i, max_lims[i] + outOfBoundPos[i], max_lims[i], reached_pos, &excededpos_reached);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK (!res, Asserter::format("check if joint %d desn't exced max limit. target was: %f reached: %f, limit %f ",  (int)jointsList[i], max_lims[i] + outOfBoundPos[i], excededpos_reached, max_lims[i]));
        //if (res)
        //{
        //    goToSingle(i, home[i], NULL); //I need to go to home in order to leave joint in safe position for further tests (other body parts)
        //    sprintf(buff, "Limit execeeded! Limit was (%.2f). Reached pos=%.2f", max_lims[i], reached_pos); ROBOTTESTINGFRAMEWORK_ASSERT_ERROR(buff);
        //}
        //else { sprintf(buff, "Test successfull, joint %d, target was: %f reached: %f, limit %f ", (int)jointsList[i], max_lims[i] + outOfBoundPos[i], reached_pos, max_lims[i]); ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff); }

    //3) Check min limit
        //check that min_limit is reachable
        sprintf(buff,"Testing if min limit is reachable, joint %d, min limit: %f",(int)jointsList[i],min_lims[i]);ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);
        res = goToSingle(i, min_lims[i], &reached_pos);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK (res, Asserter::format("joint %d moved to min limit: %f reached: %f",  (int)jointsList[i], min_lims[i], reached_pos));

        /*if(!res)
        {
            goToSingle(i,home[i], NULL);
            sprintf(buff, "Timeout while reaching desired position(%.2f). Reached pos=%.2f", min_lims[i], reached_pos);ROBOTTESTINGFRAMEWORK_ASSERT_ERROR(buff);
        }
        else { sprintf(buff, "Test successfull, joint %d, min limit: %f reached: %f", (int)jointsList[i], min_lims[i], reached_pos); ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff); }*/

    //4) check that min_limit - outOfBoundPos is NOT reachable
        sprintf(buff, "Testing that min limit cannot be exceeded, joint %d, min limit: %f", (int)jointsList[i], min_lims[i]); ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);
        res = goToSingleExceed(i, min_lims[i] - outOfBoundPos[i], min_lims[i], reached_pos, & excededpos_reached);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK (!res, Asserter::format("check if joint %d desn't exced min limit. target was: %f reached: %f, limit %f ",  (int)jointsList[i], min_lims[i] - outOfBoundPos[i], excededpos_reached, min_lims[i]));

        //if (res)
        //{
        //    goToSingle(i, home[i], NULL); //I need to go to home in order to leave joint in safe position for further tests (other body parts)
        //    sprintf(buff, "Limit execeeded! Limit was (%.2f). Reached pos=%.2f", min_lims[i], reached_pos); ROBOTTESTINGFRAMEWORK_ASSERT_ERROR(buff);
        //}
        //else { sprintf(buff, "Test successfull, joint %d, target was: %f reached: %f, limit %f ", (int)jointsList[i], min_lims[i] - outOfBoundPos[i], reached_pos, min_lims[i]); ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff); }

    //5) Check home position
        sprintf(buff,"Testing joint %d, homing to: %f",(int)jointsList[i],home[i]);ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);
        res = goToSingle(i, home[i], &reached_pos);
        if(!res)
        {
            sprintf(buff, "Timeout while reaching desired position(%.2f). Reached pos=%.2f", home[i], reached_pos);ROBOTTESTINGFRAMEWORK_ASSERT_ERROR(buff);
        }
        else{ sprintf(buff, "Homing joint %d complete", (int)jointsList[i]); ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff); }

    }
    ienc->getEncoders(enc_jnt.data());
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Test ends. All joints are going to home....");
    goTo(home);
    yarp::os::Time::delay(2.0);
}
