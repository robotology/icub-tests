// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _MOTORSTICTION_H_
#define _MOTORSTICTION_H_

#include <string>
#include <vector>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Matrix.h>

class stiction_data
{
    public:
    int    jnt;
    bool   pos_test_passed;
    bool   neg_test_passed;
    double pos_opl;
    double neg_opl;

    public:
    stiction_data() {pos_test_passed=false; neg_test_passed=false; pos_opl=0; neg_opl=0;}
};

class MotorStiction : public YarpTestCase
{
public:
    MotorStiction();
    virtual ~MotorStiction();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, yarp::os::ConstString title);
    void saveToFile(std::string filename, yarp::os::Bottle &b);

private:
    std::string robotName;
    std::string partName;
    std::vector<stiction_data> stiction_data_list;
    yarp::sig::Vector jointsList;
    yarp::sig::Vector home;
    yarp::sig::Vector opl_step;
    yarp::sig::Vector opl_max;
    yarp::sig::Vector opl_delay;
    yarp::sig::Vector movement_threshold;

    int    n_part_joints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IAmplifierControl *iamp;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IOpenLoopControl  *iopl;

};

#endif //_MOTORSTICTION_H_
