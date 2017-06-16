// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <math.h>
#include <Plugin.h>
#include "SensorsDuplicateReadings.h"
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::math;

// prepare the plugin
PREPARE_PLUGIN(SensorsDuplicateReadings)

SensorsDuplicateReadings::SensorsDuplicateReadings() : yarp::rtf::TestCase("SensorsDuplicateReadings") {
}

SensorsDuplicateReadings::~SensorsDuplicateReadings() { }

bool SensorsDuplicateReadings::setup(yarp::os::Property &property) {

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
        RTF_ASSERT_ERROR_IF(btport && btport->size()>=3, "The ports must be given as lists of <portname> <toleratedDuplicates>");
        DuplicateReadingsPortInfo info;
        info.name = btport->get(0).asString();
        info.toleratedDuplicates = btport->get(1).asInt();
        ports.push_back(info);
    }

    // opening port
    RTF_ASSERT_ERROR_IF(port.open("..."),
                        "opening port, is YARP network available?");
    return true;
}

void SensorsDuplicateReadings::tearDown() {
    // finalization goes her ...
    port.close();
}

void SensorsDuplicateReadings::run() {
    for(unsigned int i=0; i<ports.size(); i++) {
        port.reset();
        RTF_TEST_REPORT(Asserter::format("Checking port %s ...", ports[i].name.c_str()));
        bool connected = Network::connect(ports[i].name.c_str(), port.getName());
        RTF_TEST_FAIL_IF(connected,
                       Asserter::format("could not connect to remote port %s.", ports[i].name.c_str()));
        if(connected) {
            port.useCallback();
            Time::delay(testTime);
            port.disableCallback();

            RTF_TEST_REPORT(Asserter::format("Computed a total of %lu duplicates out of %lu samples.",
                            port.getTotalNrOfDuplicates(),port.getCount()));
            RTF_TEST_REPORT(Asserter::format("Maximum number of consecutive duplicates: %lu Maximum jitter: %lf ",
                                              port.getMaxNrOfDuplicates(), port.getMaxJitter()));

            RTF_TEST_FAIL_IF(port.getTotalNrOfDuplicates() > ports[i].toleratedDuplicates,
                           Asserter::format("Number of duplicates (%lu) is higher than the tolerated (%d)",
                                            port.getTotalNrOfDuplicates(),
                                            ports[i].toleratedDuplicates));

            Network::disconnect(ports[i].name.c_str(), port.getName());
        }
    }
}

void DuplicateDetector::onRead(yarp::sig::Vector& vec) {
    double tcurrent = Time::now();

    if(count == 0)
    {
        lastReading = vec;
        currentJitter = 0.0;
        currentNrOfDuplicates = 0;
        totalNrOfDuplicates = 0;
        lastNewValueTime = tcurrent;
        maxJitter = currentJitter;
        maxNrOfDuplicates = currentNrOfDuplicates;
    }
    else
    {
        // Check for duplicate data
        double diff = yarp::math::norm(vec-lastReading);
        if( diff < this->tolerance )
        {
            // duplicate ! report a duplicate
            currentNrOfDuplicates++;
            currentJitter = tcurrent - lastNewValueTime;

            totalNrOfDuplicates++;

            maxJitter = std::max(currentJitter,maxJitter);
            maxNrOfDuplicates = std::max(currentJitter,maxJitter);

        }
        else
        {
            // not duplicate! update last read value
            lastReading = vec;
            currentNrOfDuplicates = 0;
            currentJitter = 0.0;
            lastNewValueTime = tcurrent;
        }
    }

    count++;
}
