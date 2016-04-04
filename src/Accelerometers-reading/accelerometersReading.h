// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _ACCELEROMETERSREADING_H_
#define _ACCELEROMETERSREADING_H_

#include <string>
#include <vector>
#include <rtf/yarp/YarpTestCase.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>


/**
 * \ingroup icub-tests
 * Check if inertial MTB sensor ports are correctly publishing accelerometer data, i.e. the expected accelerometers
 * re publishing data, and this data is valid (not 0xffff).
 * No further check on the content of the vector is done.
 *
 *  Accepts the following parameters:
 * | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
 * |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
 * | name           | string | -     | "AccelerometersReading" | No       | The name of the test. | -     |
 * | robot          | string | -     | -             | Yes      | The name of the robot.     | e.g. icub |
 * | bus            | string | -     | -             | Yes      | CAN or ETHERNET based robot. | - |
 * | part           | string | -     | -             | Yes      | The name of the robot part. | e.g. left_arm |
 * | mtbList        | string | -     | -             | Yes      | The sensors data mapping. | ordered list of sensor IDs |
 * | sampleTime     | double | s     | -             | Yes      | The sample time of the control thread | |
 *
 */
class AccelerometersReading : public YarpTestCase {
public:
    AccelerometersReading();
    virtual ~AccelerometersReading();

    virtual bool setup(yarp::os::Property& configuration);

    virtual void tearDown();

    virtual void run();

private:
    enum busType_t {
        BUSTYPE_CAN,
        BUSTYPE_ETH,
        BUSTYPE_NTYPES,
        BUSTYPE_UNKNOWN
    };

    std::string robotName;
    busType_t busType;
    std::string portName; // source port we will read from
    yarp::os::BufferedPort<yarp::sig::Vector> port; // anonymous destination port
    yarp::os::Bottle* mtbListBottle;
    double sampleTime;
    const double sensoReadingCycles = 500; // read MTB sensors for 5s
};

#endif //_ACCELEROMETERSREADING_H_
