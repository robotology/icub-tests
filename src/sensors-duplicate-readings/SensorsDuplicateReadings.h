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

#ifndef _SENSORSDUPLICATEREADINGS_H_
#define _SENSORSDUPLICATEREADINGS_H_

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <vector>

class DuplicateReadingsPortInfo {
public:
    std::string name;
    int toleratedDuplicates;
};


class DuplicateDetector : public yarp::os::BufferedPort<yarp::sig::Vector> {
public:

    void reset() {
        count = 0;
        tolerance = 1e-12;
    }

    unsigned long getCount() { return count; }
    unsigned long getMaxNrOfDuplicates() { return maxNrOfDuplicates; }
    double getMaxJitter() { return maxJitter; }
    unsigned long getTotalNrOfDuplicates() { return totalNrOfDuplicates; }

    virtual void onRead(yarp::sig::Vector& vec);

private:
    unsigned long count;
    double tolerance;
    unsigned long currentNrOfDuplicates;
    unsigned long totalNrOfDuplicates;
    unsigned long maxNrOfDuplicates;
    double        lastNewValueTime;
    double        currentJitter;
    double        maxJitter;
    yarp::sig::Vector lastReading;
};


/**
 * \ingroup icub-tests
 * Check if a yarp port is correctly publishing unique values at each update .
 *
 * For some sensors, such as strain gauge based force sensors,
 * errors on the measurement ensure that two different readings of
 * the sensors are always different. If on a port that is publishing
 * a sensor reading you get two times exactly the same value, this could
 * mean that the same sensor reading was sent multiple times.
 * This could happen for several reasons:
 *  * The wrapper is running faster than the sensor frequency
 *    (depending on the configuration this could make sense or not)
 *  * The sensor readings are not properly read, for example because packets
 *    are lost or dropped (this usually indicate some problem in your system).
 *
 * Accepts the following parameters:
 * | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
 * |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
 * | name           | string | -     | "SensorsDuplicateReadings" | No       | The name of the test. | -     |
 * | time           | double | s     | -             | Yes      | Duration of the test for each port. | - |
 * | PORTS (group ) | Bottle | -     | -             | Yes      | List of couples of port/toleratedDuplicates with this format: (portname1, toleratedDuplicates1) (portname1, toleratedDuplicates1) | |
 *
 *
 */
class SensorsDuplicateReadings : public yarp::robottestingframework::TestCase {
public:
    SensorsDuplicateReadings();
    virtual ~SensorsDuplicateReadings();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

private:
    DuplicateDetector port;
    std::vector<DuplicateReadingsPortInfo> ports;
    double testTime;
};

#endif //_PORTSFREQUENCY_H
