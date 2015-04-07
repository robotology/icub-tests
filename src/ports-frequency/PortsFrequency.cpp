// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <math.h>
#include <Plugin.h>
#include "PortsFrequency.h"
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>

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
        port.reset();
        RTF_TEST_REPORT(Asserter::format("Checking port %s ...", ports[i].name.c_str()));
        bool connected = Network::connect(ports[i].name.c_str(), port.getName());
        RTF_TEST_CHECK(connected,
                       Asserter::format("could not connect to remote port %s.", ports[i].name.c_str()));
        if(connected) {
            port.useCallback();
            Time::delay(testTime);
            port.disableCallback();
            double freq = 1.0/port.getAvg();
            RTF_TEST_REPORT(Asserter::format("Calculated frequency %d hrz. (min: %d, max: %d)",
                            (int)freq, (int)(1.0/port.getMax()), (int)(1.0/port.getMin())));
            if(port.getSAvg() <= 0) {
                RTF_TEST_REPORT("Source frequency is not available");
            }
            else {
                RTF_TEST_REPORT(Asserter::format("Source frequency %d hrz. (min: %d, max: %d)",
                                                 (int)(1.0/port.getSAvg()), (int)(1.0/port.getSMax()), (int)(1.0/port.getSMin())));
            }
            double diff = fabs(freq - ports[i].frequency);
            RTF_TEST_CHECK(diff < ports[i].tolerance,
                           Asserter::format("Calculated frequency is outside the desired range [%d .. %d]",
                                            ports[i].frequency-ports[i].tolerance,
                                            ports[i].frequency+ports[i].tolerance));

            Network::disconnect(ports[i].name.c_str(), port.getName());
        }
    }
}

void DataPort::onRead(yarp::os::Bottle& bot) {
    double tcurrent = Time::now();
    Stamp stm;
    getEnvelope(stm);
    if(count != 0) {
        // calculating statistics
        double tdiff =  tcurrent - tprev;
        sum += tdiff;
        max = (tdiff > max) ? tdiff : max;
        min = (min<0 || min > tdiff) ? tdiff : min;

        // calculating statistics using time stamp
        tdiff = stm.getTime() - stprev;
        ssum += tdiff;
        smax = (tdiff > smax) ? tdiff : smax;
        smin = (smin<0 || smin > tdiff) ? tdiff : smin;
    }
    count++;
    tprev = tcurrent;
    stprev = stm.getTime();
}
