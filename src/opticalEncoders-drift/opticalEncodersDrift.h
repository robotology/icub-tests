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
#include <YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
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

    double* enc_jnt;
    double* enc_mot;
    double* zero_enc_mot;
    double* end_enc_mot;
    double* err_enc_mot;

    int     cycles;
    double* max;
    double* min;
    double* zero;
    double* speed;
};

#endif //_opticalEncodersDRIFT_H
