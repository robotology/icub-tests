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
#include <rtf/yarp/YarpTestCase.h>
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
    void executeCmd();
    void setMode(int desired_mode);

private:
    std::string robotName;
    std::string partName;
    int* jointsList;
    double frequency;
    double amplitude;
    double cycles;
    double tolerance;
    double sampleTime;
    double zero;
    int    n_part_joints;
    int    n_cmd_joints;
    enum cmd_mode_t
    {
      single_joint = 0,
      all_joints = 1,
      some_joints =2
    } cmd_mode;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IPositionDirect   *idir;

    double  cmd_single;
    double* cmd_tot;
    double* cmd_some;

    double* pos_tot;

    double prev_cmd;
};

#endif //_PositionDirect_H
