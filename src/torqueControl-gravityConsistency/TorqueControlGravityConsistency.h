// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef TORQUECONTROLGRAVITYCONSISTENCY_H_
#define TORQUECONTROLGRAVITYCONSISTENCY_H_

#include <string>
#include <rtf/yarp/YarpTestCase.h>
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
class TorqueControlGravityConsistency : public YarpTestCase
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
