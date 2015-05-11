// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _CAMERATEST_H_
#define _CAMERATEST_H_

#include <rtf/yarp/YarpTestCase.h>

class ExampleTest : public YarpTestCase {
public:
    ExampleTest();
    virtual ~ExampleTest();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();
};

#endif //_CAMERATEST_H
