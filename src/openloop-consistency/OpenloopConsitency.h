// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _OPENLOOPCONSITENCY_H_
#define _OPENLOOPCONSITENCY_H_

#include <string>
#include <YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

class OpenLoopConsitency : public YarpTestCase {
public:
    OpenLoopConsitency();
    virtual ~OpenLoopConsitency();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void executeCmd();
    void setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, yarp::os::ConstString title);

    void setRefOpenloop(double value);
    void verifyRefOpenloop(double value, yarp::os::ConstString title);
    void verifyOutputEqual(double value, yarp::os::ConstString title);
    void verifyOutputDiff(double value, yarp::os::ConstString title);

private:
    std::string robotName;
    std::string partName;
    int* jointsList;

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
    yarp::dev::IAmplifierControl *iamp;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IOpenLoopControl  *iopl;

    double  cmd_single;
    double* cmd_tot;
    double* cmd_some;

    double  prevcurr_single;
    double* prevcurr_tot;
    double* prevcurr_some;

    double* pos_tot;
};

#endif //_OPENLOOPCONSITENCY_H_
