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

#ifndef _TORQUECONTORLSTIFFDAMPCHECK_H_
#define _TORQUECONTORLSTIFFDAMPCHECK_H_

#include <string>
#include <yarp/robottestingframework/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>


using namespace yarp::os;

class TorqueControlStiffDampCheck : public yarp::robottestingframework::TestCase {
public:
    TorqueControlStiffDampCheck();
    virtual ~TorqueControlStiffDampCheck();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, std::string title);
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
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IAmplifierControl *iamp;
    yarp::dev::IControlMode     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::ITorqueControl    *itrq;
    yarp::dev::IImpedanceControl *iimp;

};

#endif //_TORQUECONTORLSTIFFDAMPCHECK_H
