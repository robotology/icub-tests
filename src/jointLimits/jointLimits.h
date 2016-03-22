// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _JOINTLIMITS_H_
#define _JOINTLIMITS_H_

#include <string>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Matrix.h>

/**
* \ingroup icub-tests
* Check if the software joint limits are properly set.
* The limits are set in the robot configuration files. The test asks the limits to robotInterface using the IControlLimits interface.
* The test moves each joint first to the max lim, then to the min lim, then to home.
* If the joint is unable to reach the software limits within a certain tolerance, the test fails and the joint is put back in home position.
* The timeout for each joint to reach the limit position is fixed to 20 seconds.
* The test uses an limited output to avoid to damage the joint if, for example, an hardware limit is reached before the software limit.
* If this limit is too small, the the joint may be unable to reach the limit (e.g. because of friction), so the value must be chosen accurately.
* The test assumes the the position control is properly working and the position pid is properly tuned.
* After testing the limits, this test also tries to move the joint out of the limits on puropose (adding to the joint limits the value of outOfBoundPosition).
* The test is successfull if the position move command is correctly stopped at the limit. 
*
* Example: testRunner -v -t JointLimits.dll -p "--robot icub --part head --joints ""(0 1 2)"" --home ""(0 0 0)"" --speed ""(20 20 20)"" --outputLimitPercent ""(30 30 30)"" --outOfBoundPosition ""(2 2 2)"" --tolerance 0.2"
*
* Check the following functions:
* \li IControlLimits::getLimits()

* Support functions:
* \li IPositionControl::getAxes()
* \li IPositionControl::positionMove()
* \li IPositionContol::setRefSpeeds()
* \li IEncoders::getEncoder()
* \li IEncoders::getEncoders()
* \li IPid::getPid()/IPid::setPid()
* \li IControlMode2::getControlMode()/setControlMode()
* \li IInteractionMode::getInteractionMode()/setInteractionMode()
*
*  Accepts the following parameters:
* | Parameter name     | Type   | Units | Default Value | Required | Description | Notes |
* |:------------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | robot              | string | -     | -             | Yes      | The name of the robot.     | e.g. icub |
* | part               | string | -     | -             | Yes      | The name of trhe robot part. | e.g. left_arm |
* | joints             | vector of ints | -     | - | Yes | List of joints to be tested | |
* | home               | vector of doubles of size joints | deg   | - | Yes | The home position for each joint | |
* | speed              | vector of doubles of size joints | deg/s | - | Yes | The reference speed used furing the movement | |
* | tolerance          | vector of doubles of size joints | deg   | - | Yes | The position tolerance used to check if the limit has been properly reached. | Typical value = 0.2 deg. |
* | outputLimitPercent | vector of doubles of size joints | %     | - | Yes | The maximum motor output (expressed as percentage). | Safe values can be, for example, 30%.|
* | outOfBoundPosition | vector of doubles of size joints | %     | - | Yes | This value is added the joint limit to test that a position command is not able to move out of the joint limits | Typical value 2 deg.|
*
*/

class JointLimits : public YarpTestCase {
public:
    JointLimits();
    virtual ~JointLimits();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goTo(yarp::sig::Vector position);
    bool goToSingle(int i, double pos, double *reached_pos);
    bool goToSingleExceed(int i, double position_to_reach, double limit, double reachedLimit, double *reached_pos);

    void setMode(int desired_mode);
    void saveToFile(std::string filename, yarp::os::Bottle &b);

private:
    std::string robotName;
    std::string partName;
    yarp::sig::Vector jointsList;

    double tolerance;

    int    n_part_joints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IControlLimits    *ilim;
    yarp::dev::IPidControl       *ipid;

    yarp::sig::Vector enc_jnt;
    yarp::sig::Vector max_lims;
    yarp::sig::Vector min_lims;
    yarp::sig::Vector outputLimit;
    yarp::sig::Vector outOfBoundPos;
    yarp::sig::Vector toleranceList;
    yarp::sig::Vector home;
    yarp::sig::Vector speed;

    bool pids_saved;
    yarp::dev::Pid* original_pids;
};

#endif //_JOINTLIMITS_H
