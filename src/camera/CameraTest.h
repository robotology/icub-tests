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

#ifndef _CAMERATEST_H_
#define _CAMERATEST_H_

#include <string>
#include <yarp/rtf/TestCase.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>


/**
* \ingroup icub-tests
* Check if a camera is publishing images at desired framerate.
*
*  Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | name           | string | -     | "CameraTest" | No       | The name of the test. | -     |
* | portname       | string | -     | -             | Yes      | The yarp port name of the camera to test. | - |
* | measure_time   | int    |  s  | 1             | No      | The duration of the test. |  |
* | expected_frequency | int    |  Hz  | 30           | No      | The expected framerate of the camera. |  |
* | tolerance      | int    | Number of frames | 5    | No     | The tolerance on the total number of frames read during the period (expected_frequency*measure_time) to consider the test sucessful. |  |
*
*/
class CameraTest : public yarp::rtf::TestCase {
public:
    CameraTest();
    virtual ~CameraTest();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

private:
    std::string cameraPortName;
    int measure_time;
    int expected_frequency;
    int tolerance;
    yarp::os::BufferedPort<yarp::sig::Image> port;
};

#endif //_CAMERATEST_H
