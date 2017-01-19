// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _EXAMPLE_TEST_H_
#define _EXAMPLE_TEST_H_

#include <yarp/rtf/TestCase.h>

/**
* \ingroup icub-tests
*
* This is just an example test, use
* it as a reference to implement new tests.
*
* Check the following functions:
* \li NA
*
* Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | name           | string | -     | "ExampleTest" | No       | The name of the test. | -     |
* | example        | string | -     | default value | No       | An example value. | - | 
*/
class ExampleTest : public yarp::rtf::TestCase {
public:
    ExampleTest();
    virtual ~ExampleTest();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();
};

#endif //_EXAMPLE_TEST_H
