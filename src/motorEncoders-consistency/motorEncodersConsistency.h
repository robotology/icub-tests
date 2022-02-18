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

#ifndef _OPTICALENCODERSCONSISTENCY_H_
#define _OPTICALENCODERSCONSISTENCY_H_

#include <string>
#include <yarp/robottestingframework/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

/**
* \ingroup icub-tests
* This tests checks if the motor encoder reading are consistent with the joint encoder readings.
* Since the two sensors may be placed in different places, with gearboxes or tendon transmissions in between, a (signed) factor is needed to convert the two measurements.
* The test performes a cyclic movement between two reference positions (min and max) and collects data from both the encoders during the movement.
* The test generates four text data files, which are subsequently opened to generate plots. In all figures the joint and motor plots need to be reasonably aligned.
* The four plots are:
* \li joint positions  vs motor positions
* \li joint velocities vs motor velocities
* \li joint positions (numerically derived by the test) vs joint velocities (measured by the control board)
* \li motor positions (numerically derived by the test) vs motor velocities (measured by the control board)
* The conversion formula from motor measurments (M) to joint encoder measurements (J) is the following:
* J = kinematic_mj * gearbox * M
* with kinematic_mj the joints coupling matrix and gearbox the gearbox reduction factor (e.g. 1:100)

* Example: testRunner v -t motorEncodersConsistency.dll -p "--robot icub --part left_arm --joints ""(0 1 2)"" --home ""(-30 30 10)"" --speed ""(20 20 20)"" --max ""(-20 40 20)"" --min ""(-40 20 0)"" --cycles 10 --tolerance 1.0 "
* Example: testRunner v -s "..\icub-tests\suites\encoders-icubSim.xml"

* Check the following functions:
* \li IEncoders::getEncoders()
* \li IEncoders::getEncoderSpeeds()
* \li IEncoders::getEncoderAccelerations()
* \li IMotorEncoder::getMotorEncoders()
* \li IMotorEncoder::getMotorEncoderSpeeds()
* \li IMotorEncoder::getMotorEncoderAccelerations()
* Note: Acceleration is not currently tested.

*
*  Accepts the following parameters:
* | Parameter name     | Type   | Units | Default Value | Required | Description | Notes |
* |:------------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | robot              | string | -     | -             | Yes      | The name of the robot.     | e.g. icub |
* | part               | string | -     | -             | Yes      | The name of trhe robot part. | e.g. left_arm |
* | joints             | vector of ints | -     |     - | Yes      | List of joints to be tested | |
* | home               | vector of doubles of size joints  | deg   | - | Yes | The home position for each joint | |
* | cycles             | int    | -     | 10            | No       | The number of test cycles (going from max to min position and viceversa | |
* | max                | vector of doubles of size joints  | deg   | - | Yes | The max position using during the joint movement | |
* | min                | vector of doubles of size joints  | deg   | - | Yes | The min position using during the joint movement | |
* | tolerance          | vector of doubles of size joints  | deg   | - | Yes | The tolerance used when moving from min to max reference position and viceversa | |
* | speed              | vector of doubles of size joints  | deg/s | - | Yes | The reference speed used during the movement  | |
* | matrix_size | int                                   | -     | - | Yes | The number of rows of the coupling matrix | Typical value = 4. |
* | matrix      | vector of doubles of size matrix_size | -     | - | Yes | The kinematic_mj coupling matrix | matrix is identity if joints are not coupled |
* | plotstring1 | string |      | - | Yes | The string which generates plot 1 | |
* | plotstring2 | string |      | - | Yes | The string which generates plot 2 | |
* | plotstring3 | string |      | - | Yes | The string which generates plot 3 | |
* | plotstring4 | string |      | - | Yes | The string which generates plot 4 | |

*
*/
class OpticalEncodersConsistency : public yarp::robottestingframework::TestCase {
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
    std::string getPath(const std::string& str);
    std::string robotName;
    std::string partName;
    std::string plotString1;
    std::string plotString2;
    std::string plotString3;
    std::string plotString4;

    yarp::sig::Vector jointsList;

    double tolerance;
    bool plot_enabled;

    int    n_part_joints;
    int    cycles;
     
    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl  *ipos;
    yarp::dev::IControlMode      *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IMotorEncoders    *imotenc;
    yarp::dev::IMotor            *imot;
    yarp::dev::IRemoteVariables  *ivar;

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
