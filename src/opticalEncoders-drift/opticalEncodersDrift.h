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

#ifndef _OPTICALENCODERSDRIFT_H_
#define _OPTICALENCODERSDRIFT_H_

#include <string>
#include <yarp/robottestingframework/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Matrix.h>

/**
* \ingroup icub-tests
* This tests checks if the relative encoders measurements are consistent over time, by performing cyclic movements between two reference positions (min and max).
* The test collects data during the joint motion, saves data to a text file a plots the result. If the relative encoder is working correctly, the plot should have no drift.
* Otherwise, a drift in the plot may be caused by a damaged reflective encoder/ optical disk.
* For best reliability an high number of cycles (e.g. >100) is suggested.

* example: testRunner -v -t OpticalEncodersDrift.dll -p "--robot icub --part head --joints ""(0 1 2)"" --home ""(0 0 0)" --speed "(20 20 20)" --max "(10 10 10)" --min "(-10 -10 -10)" --cycles 100 --tolerance 1.0 "
* example: testRunner -v -t OpticalEncodersDrift.dll -p "--robot icub --part head --joints ""(2)""     --home ""(0)""    --speed "(20      )" --max "(10      )" --min "(-10)"         --cycles 100 --tolerance 1.0 "

* Check the following functions:
* \li IMotorEncoder::getMotorEncoders()

*
*  Accepts the following parameters:
* | Parameter name     | Type   | Units | Default Value | Required | Description | Notes |
* |:------------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | robot              | string | -     | -             | Yes      | The name of the robot.     | e.g. icub |
* | part               | string | -     | -             | Yes      | The name of trhe robot part. | e.g. left_arm |
* | joints             | vector of ints | -     |     - | Yes      | List of joints to be tested | |
* | home               | vector of doubles of size joints  | deg   | - | Yes | The home position for each joint | |
* | cycles             | int    | -     | -             | Yes       | The number of test cycles (going from max to min position and viceversa) | Use values > 100 |
* | max                | vector of doubles of size joints  | deg   | - | Yes | The max position using during the joint movement | |
* | min                | vector of doubles of size joints  | deg   | - | Yes | The min position using during the joint movement | |
* | tolerance          | vector of doubles of size joints  | deg   | - | Yes | The tolerance used when moving from min to max reference position and viceversa | |
* | speed              | vector of doubles of size joints  | deg/s | - | Yes | The reference speed used during the movement  | |

*
*/

class OpticalEncodersDrift : public yarp::robottestingframework::TestCase {
public:
    OpticalEncodersDrift();
    virtual ~OpticalEncodersDrift();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    bool goHome();
    void setMode(int desired_mode);
    bool saveToFile(std::string filename, yarp::os::Bottle &b);

private:
    std::string robotName;
    std::string partName;
    yarp::sig::Vector jointsList;

    double tolerance;

    int    n_part_joints;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IControlMode     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IMotorEncoders    *imot;

    yarp::sig::Vector enc_jnt;
    yarp::sig::Vector enc_mot;
    yarp::sig::Vector home_enc_mot;
    yarp::sig::Vector end_enc_mot;
    yarp::sig::Vector err_enc_mot;

    int     cycles;
    yarp::sig::Vector max;
    yarp::sig::Vector min;
    yarp::sig::Vector home;
    yarp::sig::Vector speed;

    bool plot; //if true, the test runs gnuplot utility at end of test.
};

#endif //_opticalEncodersDRIFT_H
