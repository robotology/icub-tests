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

#ifndef _MOTORENCODERSSIGNCHECK_H_
#define _MOTORENCODERSSIGNCHECK_H_

//#include <string>
#include <yarp/robottestingframework/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
//#include <yarp/sig/Matrix.h>
#include "yarp/robottestingframework/JointsPosMotion.h"


/**
* \ingroup icub-tests
* This tests checks if the motor encoder readings increase when positive pwm is applayed to motor.
* This test helps you to check if RotorEncoderResolution parameter in robot's configuration files has
* correct sign.
* The test sets one joint per time in Open Loop control mode; then applies positive pwm starting with value defined in parameter "pwmStart"
* and increments pwm with step defined in parameter "pwmStep" until motor doesn't move of Posthreshold degree at least.
*
*
* Note: This test uses yarp::robottestingframework::jointsPosMotion class, a class for reduce time in developing test.
*
*
*  Accepts the following parameters:
* | Parameter name     | Type   | Units | Default Value | Required | Description | Notes |
* |:------------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | robot              | string | -     | -             | Yes      | The name of the robot.     | e.g. icub |
* | name               | string | -     | -             | No       | The name of test. | e.g. motEncSignCheck_Head |
* | part               | string | -     | -             | Yes      | The name of trhe robot part. | e.g. head |
* | joints             | vector of ints | -     |     - | Yes      | List of joints to be tested | |
* | home               | vector of doubles of size joints  | deg   | - | Yes | The home position for each joint. It should be distant from joint's limits | |
* | speed              | vector of doubles of size joints  | deg/s | - | Yes | The reference speed used during the movement  | |
* | pwmStart           | vector of doubles of size joints  | -     | - | Yes | The starting pwm applied to joint | |
* | pwmStep            | vector of doubles of size joints  | -     | - | Yes | The increment of pwm per time | |
* | pwmMax             | vector of doubles of size joints  | -     | - | Yes | The max pwm applicable | |
* | Posthreshold       | vector of doubles of size joints  | deg   | 5 | No  | The minumum movement to check if motor position increases | |
* | commandDelay       | vector of doubles of size joints  | deg   | 0.1 | No  | The delay between two SetRefOpenLooop commands consecutive | |
*
*/
class MotorEncodersSignCheck : public yarp::robottestingframework::TestCase {
public:
    MotorEncodersSignCheck();
    virtual ~MotorEncodersSignCheck();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();
    void setModeSingle(int i, int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode);
    void OplExecute(int i);

private:

    yarp::robottestingframework::jointsPosMotion *jPosMotion;

    std::string robotName;
    std::string partName;
    yarp::sig::Vector jointsList;
    yarp::sig::Vector home;
    yarp::sig::Vector opl_step;
    yarp::sig::Vector opl_max;
    yarp::sig::Vector opl_delay;
    yarp::sig::Vector max_lims;
    yarp::sig::Vector min_lims;
    yarp::sig::Vector pos_threshold;
    yarp::sig::Vector opl_start;

    int    n_part_joints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IControlMode     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IPWMControl       *ipwm;
    yarp::dev::IMotorEncoders    *imenc;
    yarp::dev::IPidControl       *ipid;
};

#endif //_opticalEncoders_H
