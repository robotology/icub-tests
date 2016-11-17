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

#ifndef _CARTESIANCONTROLSIMPLEP2PMOVEMENT_H_
#define _CARTESIANCONTROLSIMPLEP2PMOVEMENT_H_

#include <rtf/yarp/YarpTestCase.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>

/**
* \ingroup icub-tests
*
* This test verifies the point-to-point cartesian movement.
*
* Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description  | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:------------:|:-----:|
* |    arm-type    | string |   -   |      left     |    No    | left | right |   -   |
*/
class CartesianControlSimpleP2pMovementTest : public YarpTestCase
{
    yarp::dev::PolyDriver driver;

public:
    CartesianControlSimpleP2pMovementTest();
    virtual ~CartesianControlSimpleP2pMovementTest();
    virtual bool setup(yarp::os::Property& property);
    virtual void tearDown();
    virtual void run();
};

#endif
