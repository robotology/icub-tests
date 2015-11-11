// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _MOTORENCODERSSIGNCHECK_H_
#define _MOTORENCODERSSIGNCHECK_H_

//#include <string>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
//#include <yarp/sig/Matrix.h>
#include "rtf/yarp/JointsPosMotion.h"


/**
* \ingroup icub-tests
* This tests checks if the motor encoder readings increase when positive pwm is applayed to motor.
*
* ...work in progress...
*
*/
class MotorEncodersSignCheck : public YarpTestCase {
public:
    MotorEncodersSignCheck();
    virtual ~MotorEncodersSignCheck();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();
    void setModeSingle(int i, int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void OplExecute(int i);

private:

    RTF::YARP::jointsPosMotion *jPosMotion;

    std::string robotName;
    std::string partName;
    yarp::sig::Vector jointsList;
    yarp::sig::Vector home;
    yarp::sig::Vector opl_step;
    yarp::sig::Vector opl_max;
    yarp::sig::Vector opl_delay;
    yarp::sig::Vector max_lims;
    yarp::sig::Vector min_lims;

    int    n_part_joints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IOpenLoopControl  *iopl;
    yarp::dev::IMotorEncoders    *imenc;
};

#endif //_opticalEncoders_H
