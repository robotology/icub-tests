// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _OPTICALENCODERSDRIFT_H_
#define _OPTICALENCODERSDRIFT_H_

#include <string>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Matrix.h>

class OpticalEncodersDrift : public YarpTestCase {
public:
    OpticalEncodersDrift();
    virtual ~OpticalEncodersDrift();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void setMode(int desired_mode);
    void saveToFile(std::string filename, yarp::os::Bottle &b);

private:
    std::string robotName;
    std::string partName;
    yarp::sig::Vector jointsList;

    double threshold;

    int    n_part_joints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IMotorEncoders    *imot;

    yarp::sig::Vector enc_jnt;
    yarp::sig::Vector enc_mot;
    yarp::sig::Vector home_enc_mot;
    yarp::sig::Vector end_enc_mot;
    yarp::sig::Vector err_enc_mot;

    int     cycles;
    yarp::sig::Vector max;
    yarp::sig::Vector min;
    yarp::sig::Vector home;
    yarp::sig::Vector speed;
};

#endif //_opticalEncodersDRIFT_H
