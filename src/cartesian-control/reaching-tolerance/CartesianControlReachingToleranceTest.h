/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ugo Pattacini <ugo.pattacini@iit.it>
 *          Giulia Vezzani <giulia.vezzani@iit.it>
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

#ifndef _CARTESIANCONTROLREACHINGTOLERANCE_H_
#define _CARTESIANCONTROLREACHINGTOLERANCE_H_

#include <yarp/rtf/TestCase.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

/**
* \ingroup icub-tests
*
* This test verifies the point-to-point cartesian movement.
*
* Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required |  Description  | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-------------:|:-----:|
* |     robot      | string |   -   |    icubSim    |    No    |   robot name  |   -   |
* |    arm-type    | string |   -   |      left     |    No    | left or right |   -   |
*/
class CartesianControlReachingToleranceTest : public yarp::rtf::TestCase
{
    yarp::dev::PolyDriver drvCart;
    yarp::dev::PolyDriver drvJoint;

    double compute_error(const yarp::sig::Vector &xh, const yarp::sig::Vector &oh,
                         const yarp::sig::Vector &x, const yarp::sig::Vector &o);

public:
    CartesianControlReachingToleranceTest();
    virtual ~CartesianControlReachingToleranceTest();
    virtual bool setup(yarp::os::Property& property);
    virtual void tearDown();
    virtual void run();
};

#endif
