/*
 * iCub Robot Unit Tests (Robot Testing Framework)
 *
 * Copyright (C) 2015-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _OPENLOOPCONSISTENCY_H_
#define _OPENLOOPCONSISTENCY_H_

#include <string>
#include <yarp/robottestingframework/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

class OpenLoopConsistency : public yarp::robottestingframework::TestCase {
public:
    OpenLoopConsistency();
    virtual ~OpenLoopConsistency();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void executeCmd();
    void setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, std::string title);

    void setRefOpenloop(double value);
    void verifyRefOpenloop(double value, std::string title);
    void verifyOutputEqual(double value, std::string title);
    void verifyOutputDiff(double value, std::string title);

private:
    std::string robotName;
    std::string partName;
    int* jointsList;

    double *home;
    int    n_part_joints;
    int    n_cmd_joints;
    enum cmd_mode_t
    {
      single_joint = 0,
      all_joints = 1,
      some_joints =2
    } cmd_mode;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IAmplifierControl *iamp;
    yarp::dev::IControlMode     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IPWMControl       *ipwm;

    double  cmd_single;
    double* cmd_tot;
    double* cmd_some;

    double  prevcurr_single;
    double* prevcurr_tot;
    double* prevcurr_some;

    double* pos_tot;
};

#endif //_OPENLOOPCONSISTENCY_H_
