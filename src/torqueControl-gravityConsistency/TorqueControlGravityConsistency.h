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

#ifndef TORQUECONTROLGRAVITYCONSISTENCY_H_
#define TORQUECONTROLGRAVITYCONSISTENCY_H_

#include <string>
#include <yarp/robottestingframework/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

/**
 * \ingroup icub-tests
 * The test is supposed to be run with the iCub fixed to the pole, with the pole
 * leveled with respect to the gravity (please check this with a level before running
 * the test) and with the wholeBodyDynamics(Tree) running.
 *
 * The tests opens the wholeBodyInterface (to be migrated to use material
 * available on YARP and iDynTree) and compares the gravity compensation torque
 * coming from the model and assuming that the gravity is fixed in the based
 * with the joint torques measured by iCub (that actually come from the wholeBodyDynamics(Tree) ).
 *
 * Example: testRunner -v -t TorqueControlGravityConsistency.dll -p ""
 *
 */
class TorqueControlGravityConsistency : public yarp::robottestingframework::TestCase
{
public:
    TorqueControlGravityConsistency();

    virtual ~TorqueControlGravityConsistency();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

private:
    yarpWbi::yarpWholeBodyInterface * yarpRobot;
};

#endif
