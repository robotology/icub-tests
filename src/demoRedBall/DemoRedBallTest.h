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

#ifndef _DEMOREDBALL_H_
#define _DEMOREDBALL_H_

#include <string>
#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/Property.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>

/**
* \ingroup icub-tests
*
* This test verifies the point-to-point cartesian movement.
*
* Accepts the following parameters:
* | Parameter name | Type   | Units |  Default Value   | Required |  Description  | Notes |
* |:--------------:|:------:|:-----:|:----------------:|:--------:|:-------------:|:-----:|
* |     context    | string |   -   |  demoRedBall     |    No    |   context containing the demoRedBall conf file  |   -   |
* |      from      | string |   -   |  config-test.ini |    No    |   demoRedBall configuration file  |   -   |
*
* You can watch a <a
* href="https://www.youtube.com/watch?v=ackQ5Bfk9jk">video</a>
* of how this test will be performing.
*/
class DemoRedBallTest : public yarp::robottestingframework::TestCase
{
    struct {
        std::string robot;
        std::string eye;
        double reach_tol;
        bool use_torso;
        bool use_left;
        bool use_right;
        yarp::sig::Vector home_arm;
    } params;

    yarp::dev::PolyDriver drvJointArmL;
    yarp::dev::PolyDriver drvJointArmR;
    yarp::dev::PolyDriver drvJointTorso;
    yarp::dev::PolyDriver drvJointHead;
    yarp::dev::PolyDriver drvCartArmL;
    yarp::dev::PolyDriver drvCartArmR;
    yarp::dev::PolyDriver drvGaze;

    struct {
    yarp::dev::ICartesianControl *iarm;
    yarp::dev::IEncoders         *ienc;
    } arm_under_test;

    yarp::os::PeriodicThread *redBallPos;
    void testBallPosition(const yarp::sig::Vector &pos);

public:
    DemoRedBallTest();
    virtual ~DemoRedBallTest();
    virtual bool setup(yarp::os::Property& property);
    virtual void tearDown();
    virtual void run();
};

#endif
