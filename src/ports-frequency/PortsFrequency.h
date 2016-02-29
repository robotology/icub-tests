// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _PORTSFREQUENCY_H_
#define _PORTSFREQUENCY_H_

#include <rtf/yarp/YarpTestCase.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <vector>

class MyPortInfo {
public:
    std::string name;
    unsigned int frequency;
    unsigned int tolerance;
};


class DataPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
public:
    void reset() {
        max = smax = sum = ssum = dmax = dsum = 0.0;
        min = smin = dmin = -1.0;
        tprev = stprev = 0.0;
        count = 0;
        prevPacketCount = 0;
        packetLostCount = 0;
    }

    double getMax() { return max; }
    double getMin() { return min; }
    double getAvg() { return sum/count; }
    double getSMax() { return smax; }
    double getSMin() { return smin; }
    double getSAvg() { return ssum/count; }
    double getDMax() { return dmax; }
    double getDMin() { return dmin; }
    double getDAvg() { return dsum/count; }
    unsigned long getPacketLostCount() { return packetLostCount; }
    unsigned long getCount() { return count; }

    virtual void onRead(yarp::os::Bottle& bot);

private:
    unsigned long count, packetLostCount;
    unsigned long prevPacketCount;
    double tprev, stprev;
    double max, min, sum;       // receiver time
    double smax, smin, ssum;    // sender time
    double dmax, dmin, dsum;    // time delay
};

class PortsFrequency : public YarpTestCase {
public:
    PortsFrequency();
    virtual ~PortsFrequency();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

private:
    DataPort port;
    std::vector<MyPortInfo> ports;
    double testTime;
};

#endif //_PORTSFREQUENCY_H
