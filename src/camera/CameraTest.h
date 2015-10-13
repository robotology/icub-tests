// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _CAMERATEST_H_
#define _CAMERATEST_H_

#include <string>
#include <rtf/yarp/YarpTestCase.h>
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
class CameraTest : public YarpTestCase {
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
