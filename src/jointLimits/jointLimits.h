// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _JOINTLIMITS_H_
#define _JOINTLIMITS_H_

#include <string>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Matrix.h>

class JointLimits : public YarpTestCase {
public:
    JointLimits();
    virtual ~JointLimits();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goTo(yarp::sig::Vector position);
    bool goToSingle(int i, double pos, double *reached_pos);

    void setMode(int desired_mode);
    void saveToFile(std::string filename, yarp::os::Bottle &b);

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
    yarp::dev::IControlLimits    *ilim;
    yarp::dev::IPidControl       *ipid;

    yarp::sig::Vector enc_jnt;
    yarp::sig::Vector max_lims;
    yarp::sig::Vector min_lims;
    yarp::sig::Vector outputLimit;

    yarp::sig::Vector home;
    yarp::sig::Vector speed;

    bool pids_saved;
    yarp::dev::Pid* original_pids;
};

#endif //_JOINTLIMITS_H
