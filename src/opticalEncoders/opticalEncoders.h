// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _OPTICALENCODERS_H_
#define _OPTICALENCODERS_H_

#include <string>
#include <YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

class OpticalEncoders : public YarpTestCase {
public:
    OpticalEncoders();
    virtual ~OpticalEncoders();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void setMode(int desired_mode);

private:
    std::string robotName;
    std::string partName;
    
    yarp::sig::Vector jointsList;

    double tolerance;

    int    n_part_joints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IMotorEncoders    *imot;

    double* enc_jnt;
    yarp::sig::Vector enc_jnt2mot;
    double* enc_mot;
    yarp::sig::Vector enc_mot2jnt;
    double* spd_jnt;
    yarp::sig::Vector spd_jnt2mot;
    double* spd_mot;
    yarp::sig::Vector spd_mot2jnt;
    double* acc_jnt;
    yarp::sig::Vector acc_jnt2mot;
    double* acc_mot;
    yarp::sig::Vector acc_mot2jnt;

    double* max;
    double* min;
    double* zero;
    double* speed;

    yarp::sig::Matrix matrix_arms;
    yarp::sig::Matrix matrix_torso;
    yarp::sig::Matrix matrix_legs;
    yarp::sig::Matrix matrix_head;

    yarp::sig::Matrix matrix;
    yarp::sig::Matrix inv_matrix;
    yarp::sig::Matrix trasp_matrix;
    yarp::sig::Matrix inv_trasp_matrix;
};

#endif //_opticalEncoders_H
