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

#ifndef _PORTSFREQUENCY_H_
#define _PORTSFREQUENCY_H_

#include <yarp/robottestingframework/TestCase.h>
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

class PortsFrequency : public yarp::robottestingframework::TestCase {
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
