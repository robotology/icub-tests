// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _POSITIONDIRECT_H_
#define _POSITIONDIRECT_H_

#include <string>
#include <YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

class PositionDirect : public YarpTestCase {
public:
    PositionDirect();
    virtual ~PositionDirect();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();

private:
    std::string robotName;
    std::string partName;
    double* jointsList;
    double* currPos;
    double frequency;
    double amplitude;
    double cycles;
    double tolerance;
    double sampleTime;
    double zero;
    int    njoints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IPositionDirect   *idir;
};

#endif //_PositionDirect_H
