/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef _DEMOREDBALL_H_
#define _DEMOREDBALL_H_

#include <string>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
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
* of how this test will be running.
*/
class DemoRedBallTest : public YarpTestCase
{
    struct {
        std::string robot;
        std::string eye;
        double reach_tol;
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

    yarp::os::RateThread *redBallPos;
    void testBallPosition(const yarp::sig::Vector &pos);

public:
    DemoRedBallTest();
    virtual ~DemoRedBallTest();
    virtual bool setup(yarp::os::Property& property);
    virtual void tearDown();
    virtual void run();
};

#endif
