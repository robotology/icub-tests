// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <math.h>
#include <rtf/dll/Plugin.h>
#include "PortsFrequency.h"
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/QosStyle.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;

// prepare the plugin
PREPARE_PLUGIN(PortsFrequency)

PortsFrequency::PortsFrequency() : YarpTestCase("PortsFrequency") {
}

PortsFrequency::~PortsFrequency() { }

bool PortsFrequency::setup(yarp::os::Property &property) {

    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
   testTime = (property.check("time")) ? property.find("time").asDouble() : 2;

    RTF_ASSERT_ERROR_IF(property.check("PORTS"),
                        "A list of the ports must be given");

    yarp::os::Bottle portsSet = property.findGroup("PORTS").tail();
    for(unsigned int i=0; i<portsSet.size(); i++) {
        yarp::os::Bottle* btport = portsSet.get(i).asList();
        RTF_ASSERT_ERROR_IF(btport && btport->size()>=3, "The ports must be given as lists of <portname> <grequency> <tolerance>");
        MyPortInfo info;
        info.name = btport->get(0).asString();
        info.frequency = btport->get(1).asInt();
        info.tolerance = btport->get(2).asInt();
        ports.push_back(info);
    }

    // opening port
    RTF_ASSERT_ERROR_IF(port.open("..."),
                        "opening port, is YARP network available?");
    return true;
}

void PortsFrequency::tearDown() {
    // finalization goes her ...
    port.close();
}

void PortsFrequency::run() {
    for(unsigned int i=0; i<ports.size(); i++) {
        RTF_TEST_REPORT("");
        port.reset();
        RTF_TEST_REPORT(Asserter::format("Checking port %s ...", ports[i].name.c_str()));
        bool connected = Network::connect(ports[i].name.c_str(), port.getName());
        RTF_TEST_FAIL_IF(connected,
                       Asserter::format("could not connect to remote port %s.", ports[i].name.c_str()));
        if(connected) {
            // setting QOS
            QosStyle qos;
            qos.setPacketPriorityByLevel(QosStyle::PacketPriorityHigh);
            qos.setThreadPriority(30);
            qos.setThreadPolicy(1);
            Network::setConnectionQos(ports[i].name.c_str(), port.getName(), qos);
            port.useCallback();
            Time::delay(testTime);
            port.disableCallback();
            if(port.getSAvg() <= 0) {
                RTF_TEST_REPORT("Sender frequency is not available");
            }
            else {
                RTF_TEST_REPORT(Asserter::format("Time delay between sender/receiver is %.4f s. (min: %.4f, max: %.f4)",
                                port.getDAvg(), port.getDMax(), port.getDMin()));
                RTF_TEST_REPORT(Asserter::format("Sender frequency %d hrz. (min: %d, max: %d)",
                                                 (int)(1.0/port.getSAvg()), (int)(1.0/port.getSMax()), (int)(1.0/port.getSMin())));
            }
            double freq = 1.0/port.getAvg();
            RTF_TEST_REPORT(Asserter::format("Receiver frequency %d hrz. (min: %d, max: %d)",
                            (int)freq, (int)(1.0/port.getMax()), (int)(1.0/port.getMin())));
            double diff = fabs(freq - ports[i].frequency);
            RTF_TEST_FAIL_IF(diff < ports[i].tolerance,
                           Asserter::format("Receiver frequency is outside the desired range [%d .. %d]",
                                            ports[i].frequency-ports[i].tolerance,
                                            ports[i].frequency+ports[i].tolerance));
            RTF_TEST_REPORT(Asserter::format("Lost %ld packets. received (%ld)", 
                                             port.getPacketLostCount(), port.getCount()));
            Network::disconnect(ports[i].name.c_str(), port.getName());
        }
    }
}

void DataPort::onRead(yarp::os::Bottle& bot) {
    double tcurrent = Time::now();
    Stamp stm;
    bool hasTimeStamp = getEnvelope(stm);

    if(count == 0) {
        if(hasTimeStamp) {
            double tdiff = fabs(tcurrent - stm.getTime());
            dsum = dmax = dmin = tdiff;
            prevPacketCount = stm.getCount();
        }
    }
    else {
        // calculating statistics
        double tdiff =  fabs(tcurrent - tprev);
        sum += tdiff;
        max = (tdiff > max) ? tdiff : max;
        min = (min<0 || min > tdiff) ? tdiff : min;

        // calculating statistics using time stamp
        if(hasTimeStamp) {
            tdiff = fabs(stm.getTime() - stprev);
            ssum += tdiff;
            smax = (tdiff > smax) ? tdiff : smax;
            smin = (smin<0 || smin > tdiff) ? tdiff : smin;

            // calculating time delay
            double tdiff = fabs(tcurrent - stm.getTime());
            dsum += tdiff;
            dmax = (tdiff > dmax) ? tdiff : dmax;
            dmin = (dmin<0 || dmin > tdiff) ? tdiff : dmin;
            // calculating packet losts
            if(stm.getCount() > prevPacketCount)
                packetLostCount += stm.getCount() - prevPacketCount - 1;
            prevPacketCount = stm.getCount();
        }
    }

    count++;
    tprev = tcurrent;
    if(hasTimeStamp)
        stprev = stm.getTime();
}
