// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _SENSORSDUPLICATEREADINGS_H_
#define _SENSORSDUPLICATEREADINGS_H_

#include <rtf/yarp/YarpTestCase.h>
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
 *
 *
 */
class SensorsDuplicateReadings : public YarpTestCase {
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
