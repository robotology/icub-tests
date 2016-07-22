// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _TORQUECONTORLSTIFFDAMPCHECK_H_
#define _TORQUECONTORLSTIFFDAMPCHECK_H_

#include <string>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>


using namespace yarp::os;

class TorqueControlStiffDampCheck : public YarpTestCase {
public:
    TorqueControlStiffDampCheck();
    virtual ~TorqueControlStiffDampCheck();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, yarp::os::ConstString title);
    bool setAndCheckImpedance(int joint, double stiffness, double damping);
    void saveToFile(std::string filename, yarp::os::Bottle &b);
    std::string getPath(const std::string& str);

private:
    std::string robotName;
    std::string partName;
    int* jointsList;
    int    n_part_joints;
    int    n_cmd_joints;
    double *stiffness;
    double *damping;
    double *home;
    double *pos_tot;
    double  testLen_sec;
    Bottle b_pos_trq;
    Bottle b_vel_trq;
    bool plot_enabled;


    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl2 *ipos;
    yarp::dev::IAmplifierControl *iamp;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::ITorqueControl    *itrq;
    yarp::dev::IImpedanceControl *iimp;

};

#endif //_TORQUECONTORLSTIFFDAMPCHECK_H
