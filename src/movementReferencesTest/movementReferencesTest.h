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

#ifndef _MOVEMENTREFERNCESTEST_
#define _MOVEMENTREFERNCESTEST_

#include <yarp/rtf/TestCase.h>

#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include "yarp/rtf/JointsPosMotion.h"

/**
* \ingroup icub-tests
* Check IPositionControl, IVelocityControl, IPWMControl, IPositionDirect.
*
* Check the following functions:
* \li IPositionControl::getPositionTarget()
* \li IVelocityControl::getRefVelocity()
* \li IPositionDirect::getRefPosition()
* \li IPWMControl::getRefDutyCycle()
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
class MovementReferencesTest : public yarp::rtf::TestCase {
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
    yarp::dev::IPositionControl *iPosition;
    yarp::dev::IPWMControl *iPWM;
    yarp::dev::IPositionDirect *iPosDirect;
    yarp::dev::IControlMode *iControlMode;
    yarp::dev::IVelocityControl *iVelocity;
    
    
    
    bool initialized;
    std::string robotName;
    std::string partName;

    int numJointsInPart;
    int numJoints;
    yarp::rtf::jointsPosMotion *jPosMotion;
    yarp::sig::Vector jointsList;
    int *jList;
    
    yarp::sig::Vector targetPos;
    yarp::sig::Vector homePos;
    yarp::sig::Vector refVel;
    yarp::sig::Vector refAcc;
    //yarp::sig::Vector timeout;
};

#endif //_MOVEMENTREFERNCESTEST_
