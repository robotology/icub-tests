// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _MOVEMENTREFERNCESTEST_
#define _MOVEMENTREFERNCESTEST_

#include <rtf/yarp/YarpTestCase.h>

#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include "rtf/yarp/JointsPosMotion.h"

/**
* \ingroup icub-tests
* Check IPositionControl2, IVelocityControl2, IOpenLoopControl, IPositionDirect.
*
* Check the following functions:
* \li IPositionControl2::getPositionTarget()
* \li IVelocityControl2::getRefVelocity()
* \li IPositionDirect::getRefPosition()
* \li IOpenLoopControl::getRefOutput()
*
*
*  Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | name           | string | -     | "MovementReferencesTest" | No       | The name of the test. | -     |
* | portname       | string | -     | -             | Yes      | The yarp port name of the controlboard to test. | - |
* | joints         | int    | -     | -             | Yes      | Number of axes in the controlboard. | Must be consistent with the value returned by getAxes method. |
* | home           | vector of doubles of size joints | deg | - | Yes  | For each joint the position to reach for passing the test. | |
* | target         | vector of doubles of size joints | deg | - | Yes  | For each joint the position to reach for passing the test. | |
* | refvel         | vector of doubles of size joints | deg/s | - | Yes | For each joint the reference velocity value to set in the low level trajectory generator. | |
* | refacc         | vector of doubles of size joints | deg/s^2 | - | No | For each joint the reference acceleration value to set in the low level trajectory generator. | |
*
*/
class MovementReferencesTest : public YarpTestCase {
public:
    MovementReferencesTest();
    virtual ~MovementReferencesTest();

    virtual bool setup(yarp::os::Property& configuration);

    virtual void tearDown();

    virtual void run();

private:
    void setAndCheckControlMode(int j, int mode);


    yarp::dev::PolyDriver *dd;
    yarp::dev::IEncoders *iEncoders;
    yarp::dev::IPositionControl2 *iPosition2;
    yarp::dev::IOpenLoopControl *iOpenLoop;
    yarp::dev::IPositionDirect *iPosDirect;
    yarp::dev::IControlMode2 *iControlMode2;
    yarp::dev::IVelocityControl2 *iVelocity2;
    
    
    
    bool initialized;
    std::string robotName;
    std::string partName;

    int numJointsInPart;
    int numJoints;
    RTF::YARP::jointsPosMotion *jPosMotion;
    yarp::sig::Vector jointsList;
    int *jList;
    
    yarp::sig::Vector targetPos;
    yarp::sig::Vector homePos;
    yarp::sig::Vector refVel;
    yarp::sig::Vector refAcc;
    //yarp::sig::Vector timeout;
};

#endif //_MOVEMENTREFERNCESTEST_
