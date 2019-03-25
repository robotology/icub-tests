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
