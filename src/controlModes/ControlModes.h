// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _CONTROLMODES_H_
#define _CONTROLMODES_H_

#include <string>
<<<<<<< HEAD
#include <rtf/yarp/YarpTestCase.h>
=======
#include <YarpTestCase.h>
>>>>>>> origin/master
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

class ControlModes : public YarpTestCase {
public:
    ControlModes();
    virtual ~ControlModes();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void executeCmd();
    void setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, yarp::os::ConstString title);
    void verifyAmplifier(int desired_amplifier_mode, yarp::os::ConstString title);

    void zeroCurrentLimits();
    void getOriginalCurrentLimits();
    void resetOriginalCurrentLimits();

private:
    std::string robotName;
    std::string partName;
    int* jointsList;

    double zero;
    int    n_part_joints;
    int    n_cmd_joints;
    enum cmd_mode_t
<<<<<<< HEAD
    {
=======
    { 
>>>>>>> origin/master
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
    yarp::dev::IPositionDirect   *idir;
    yarp::dev::IVelocityControl  *ivel;
    yarp::dev::ITorqueControl    *itrq;

    double  cmd_single;
    double* cmd_tot;
    double* cmd_some;

    double  prevcurr_single;
    double* prevcurr_tot;
    double* prevcurr_some;

    double* pos_tot;
};

#endif //_CONTROLMODES_H
