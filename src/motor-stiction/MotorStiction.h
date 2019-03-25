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

#ifndef _MOTORSTICTION_H_
#define _MOTORSTICTION_H_

#include <string>
#include <vector>
#include <yarp/rtf/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Matrix.h>

class stiction_data
{
    public:
    int    jnt;
    int    cycle;
    bool   pos_test_passed;
    bool   neg_test_passed;
    double pos_opl;
    double neg_opl;

    public:
    stiction_data() {jnt=0; cycle=0; pos_test_passed=false; neg_test_passed=false; pos_opl=0; neg_opl=0;}
};

class MotorStiction : public yarp::rtf::TestCase
{
public:
    MotorStiction();
    virtual ~MotorStiction();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    void goHome();
    void setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void setModeSingle(int i, int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, std::string title);
    void saveToFile(std::string filename, yarp::os::Bottle &b);
    
    //ok if the joints moves of 5 degrees
    void OplExecute(int i, std::vector<yarp::os::Bottle>& dataToPlotList, stiction_data& current_test, bool positive_sign);
    
    //ok if the joint reaches the hardware limit
    void OplExecute2(int i, std::vector<yarp::os::Bottle>& dataToPlotList, stiction_data& current_test, bool positive_sign);

private:
    std::string robotName;
    std::string partName;
    int repeat;
    std::vector<stiction_data> stiction_data_list;
    yarp::sig::Vector jointsList;
    yarp::sig::Vector home;
    yarp::sig::Vector opl_step;
    yarp::sig::Vector opl_max;
    yarp::sig::Vector opl_delay;
    yarp::sig::Vector movement_threshold;
    yarp::sig::Vector max_lims;
    yarp::sig::Vector min_lims;

    int    n_part_joints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IAmplifierControl *iamp;
    yarp::dev::IControlMode     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IPWMControl       *ipwm;
    yarp::dev::IControlLimits    *ilim;
};

#endif //_MOTORSTICTION_H_
