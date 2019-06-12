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

#ifndef _MOTORTEST_H_
#define _MOTORTEST_H_

#include <yarp/robottestingframework/TestCase.h>

#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

/**
* \ingroup icub-tests
* Check IPositionControl and IEncoders.
*
* Check the following functions:
* \li IPositionControl::getAxes()
* \li IPositionControl::positionMove()
* \li IPositionControl::checkMotionDone()
* \li IPositionContol::setRefSpeeds()
* \li IPositionControl::setRefAccelerations()
* \li IEncoders::getEncoder()
* \li IEncoders::getEncoders()
*
*  Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | name           | string | -     | "MotorTest" | No       | The name of the test. | -     |
* | portname       | string | -     | -             | Yes      | The yarp port name of the controlboard to test. | - |
* | joints         | int    | -     | -             | Yes      | Number of axes in the controlboard. | Must be consistent with the value returned by getAxes method. |
* | target         | vector of doubles of size joints | deg | - | Yes  | For each joint the position to reach for passing the test. | |
* | min            | vector of doubles of size joints | deg | - | Yes  | For each joint the maximum lower error with respect to the target to consider the test as successful. | |
* | max            | vector of doubles of size joints | deg | - | Yes  | For each joint the maximum upper error with respect to the target to consider the test as successful. | |
* | refvel         | vector of doubles of size joints | deg/s | - | Yes | For each joint the reference velocity value to set in the low level trajectory generator. | |
* | refacc         | vector of doubles of size joints | deg/s^2 | - | No | For each joint the reference acceleration value to set in the low level trajectory generator. | |
* | timeout         | vector of doubles of size joints | s | - | Yes | For each joint the maximum time to wait for the joint to reach the target. | |
*
*/
class MotorTest : public yarp::robottestingframework::TestCase {
public:
    MotorTest();
    virtual ~MotorTest();

    virtual bool setup(yarp::os::Property& configuration);

    virtual void tearDown();

    virtual void run();

private:
    yarp::dev::PolyDriver m_driver;
    yarp::dev::IEncoders *iEncoders;
    yarp::dev::IPositionControl *iPosition;
    bool m_initialized;
    std::string m_portname;
    int m_NumJoints;
    double *m_aTargetVal;
    double *m_aHome;
    double *m_aMaxErr;
    double *m_aMinErr;
    double *m_aRefVel;
    double *m_aRefAcc;
    double *m_aTimeout;
};

#endif //_MOTORTEST_H_
