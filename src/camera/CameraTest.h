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

class CameraTest : public YarpTestCase {
public:
    CameraTest();
    virtual ~CameraTest();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

private:
    std::string cameraPortName;
    int times;
    int frequency;
    int tolerance;
    yarp::os::BufferedPort<yarp::sig::Image> port;
};

#endif //_CAMERATEST_H
