// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _FTSENSORTEST_H_
#define _FTSENSORTEST_H_

#include <yarp/rtf/TestCase.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>


/**
* \ingroup icub-tests
* Check if a FT sensor port is correctly publishing a vector with 6 values.
* No further check on the content of the vector is done.
*
*  Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | name           | string | -     | "FtSensorTest" | No       | The name of the test. | -     |
* | portname       | string | -     | -             | Yes      | The yarp port name of the FT sensor to test. | - |
*
*/
class FtSensorTest : public yarp::rtf::TestCase {
public:
    FtSensorTest();
    virtual ~FtSensorTest();

    virtual bool setup(yarp::os::Property& configuration);

    virtual void tearDown();

    virtual void run();

private:
    yarp::os::BufferedPort<yarp::sig::Vector> port;
    std::string portname;
};

#endif //_FTSENSORTEST_H_
