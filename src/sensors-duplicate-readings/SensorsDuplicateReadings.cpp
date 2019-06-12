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

#include <math.h>
#include <Plugin.h>
#include "SensorsDuplicateReadings.h"
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::math;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(SensorsDuplicateReadings)

SensorsDuplicateReadings::SensorsDuplicateReadings() : yarp::robottestingframework::TestCase("SensorsDuplicateReadings") {
}

SensorsDuplicateReadings::~SensorsDuplicateReadings() { }

bool SensorsDuplicateReadings::setup(yarp::os::Property &property) {

    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
   testTime = (property.check("time")) ? property.find("time").asDouble() : 2;

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF(property.check("PORTS"),
                        "A list of the ports must be given");

    yarp::os::Bottle portsSet = property.findGroup("PORTS").tail();
    for(unsigned int i=0; i<portsSet.size(); i++) {
        yarp::os::Bottle* btport = portsSet.get(i).asList();
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF(btport && btport->size()>=3, "The ports must be given as lists of <portname> <toleratedDuplicates>");
        DuplicateReadingsPortInfo info;
        info.name = btport->get(0).asString();
        info.toleratedDuplicates = btport->get(1).asInt();
        ports.push_back(info);
    }

    // opening port
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF(port.open("..."),
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
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Checking port %s ...", ports[i].name.c_str()));
        bool connected = Network::connect(ports[i].name.c_str(), port.getName());
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF(connected,
                       Asserter::format("could not connect to remote port %s.", ports[i].name.c_str()));
        if(connected) {
            port.useCallback();
            Time::delay(testTime);
            port.disableCallback();

            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Computed a total of %lu duplicates out of %lu samples.",
                            port.getTotalNrOfDuplicates(),port.getCount()));
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Maximum number of consecutive duplicates: %lu Maximum jitter: %lf ",
                                              port.getMaxNrOfDuplicates(), port.getMaxJitter()));

            ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF(port.getTotalNrOfDuplicates() > ports[i].toleratedDuplicates,
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
