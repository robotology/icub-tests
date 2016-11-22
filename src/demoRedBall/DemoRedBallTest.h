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

#include <rtf/yarp/YarpTestCase.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/PolyDriver.h>

/**
* \ingroup icub-tests
*
* This test verifies the point-to-point cartesian movement.
*
* Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required |  Description  | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-------------:|:-----:|
* |     context    | string |   -   |  demoRedBall  |    No    |   context containing the demoRedBall conf file  |   -   |
* |      from      | string |   -   |  config.ini   |    No    |   demoRedBall configuration file  |   -   |
*/
class DemoRedBallTest : public YarpTestCase
{
    yarp::dev::PolyDriver drvJointArmL;
    yarp::dev::PolyDriver drvJointArmR;
    yarp::dev::PolyDriver drvJointTorso;
    yarp::dev::PolyDriver drvJointHead;
    yarp::dev::PolyDriver drvCartArmL;
    yarp::dev::PolyDriver drvCartArmR;
    yarp::dev::PolyDriver drvGaze;

    yarp::os::RateThread *redBallPos;

public:
    DemoRedBallTest();
    virtual ~DemoRedBallTest();
    virtual bool setup(yarp::os::Property& property);
    virtual void tearDown();
    virtual void run();
};

#endif
