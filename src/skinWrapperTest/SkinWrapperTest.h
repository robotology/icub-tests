/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Nicolo' Genesio
 * email:  nicolo.genesio@iit.it
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

#ifndef _SKINWRAPPERTEST_H_
#define _SKINWRAPPERTEST_H_

#include <rtf/yarp/YarpTestCase.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>

/**
* \ingroup icub-tests
*
* This test verifies the functionalities of skinWrapper after the removal of analogServer from icub-main.
*
*/
class SkinWrapperTest : public YarpTestCase
{
    yarp::dev::PolyDriver dd1;
    yarp::dev::PolyDriver dd2;
    yarp::dev::PolyDriver dd3;
    yarp::dev::PolyDriver dd4;

public:
    SkinWrapperTest();
    virtual ~SkinWrapperTest();
    virtual bool setup(yarp::os::Property& property);
    virtual void tearDown();
    virtual void run();
};

#endif
