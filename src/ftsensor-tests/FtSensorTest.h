// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _FTSENSORTEST_H_
#define _FTSENSORTEST_H_

#include <YarpTestCase.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

class FtSensorTest : public YarpTestCase {
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
