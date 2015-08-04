// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _OPTICALENCODERSCONSISTENCY_H_
#define _OPTICALENCODERSCONSISTENCY_H_

#include <string>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

class OpticalEncodersConsistency : public YarpTestCase {
public:
    OpticalEncodersConsistency();
    virtual ~OpticalEncodersConsistency();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void setMode(int desired_mode);
    void saveToFile(std::string filename, yarp::os::Bottle &b);

private:
    std::string robotName;
    std::string partName;
    std::string plotString1;
    std::string plotString2;
    std::string plotString3;
    std::string plotString4;

    yarp::sig::Vector jointsList;

    double tolerance;
    double position_move_tolerance;

    int    n_part_joints;
    int    cycles;
    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IMotorEncoders    *imotenc;
    yarp::dev::IMotor            *imot;

    yarp::sig::Vector zero_vector;
    yarp::sig::Vector enc_jnt;
    yarp::sig::Vector enc_jnt2mot;
    yarp::sig::Vector enc_mot;
    yarp::sig::Vector enc_mot2jnt;
    yarp::sig::Vector vel_jnt;
    yarp::sig::Vector vel_jnt2mot;
    yarp::sig::Vector vel_mot;
    yarp::sig::Vector vel_mot2jnt;
    yarp::sig::Vector acc_jnt;
    yarp::sig::Vector acc_jnt2mot;
    yarp::sig::Vector acc_mot;
    yarp::sig::Vector acc_mot2jnt;

    yarp::sig::Vector prev_enc_jnt;
    yarp::sig::Vector prev_enc_jnt2mot;
    yarp::sig::Vector prev_enc_mot;
    yarp::sig::Vector prev_enc_mot2jnt;
    yarp::sig::Vector prev_vel_jnt;
    yarp::sig::Vector prev_vel_jnt2mot;
    yarp::sig::Vector prev_vel_mot;
    yarp::sig::Vector prev_vel_mot2jnt;
    yarp::sig::Vector prev_acc_jnt;
    yarp::sig::Vector prev_acc_jnt2mot;
    yarp::sig::Vector prev_acc_mot;
    yarp::sig::Vector prev_acc_mot2jnt;

    yarp::sig::Vector diff_enc_jnt;
    yarp::sig::Vector diff_enc_jnt2mot;
    yarp::sig::Vector diff_enc_mot;
    yarp::sig::Vector diff_enc_mot2jnt;
    yarp::sig::Vector diff_vel_jnt;
    yarp::sig::Vector diff_vel_jnt2mot;
    yarp::sig::Vector diff_vel_mot;
    yarp::sig::Vector diff_vel_mot2jnt;
    yarp::sig::Vector diff_acc_jnt;
    yarp::sig::Vector diff_acc_jnt2mot;
    yarp::sig::Vector diff_acc_mot;
    yarp::sig::Vector diff_acc_mot2jnt;

    yarp::sig::Vector max;
    yarp::sig::Vector min;
    yarp::sig::Vector home;
    yarp::sig::Vector speed;
    yarp::sig::Vector gearbox;

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
