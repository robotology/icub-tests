// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <cstdlib>
#include <sstream>
#include <rtf/dll/Plugin.h>
#include <rtf/Asserter.h>

#include <yarp/os/Time.h>

#include "accelerometersReading.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;

// prepare the plugin
PREPARE_PLUGIN(AccelerometersReading)

AccelerometersReading::AccelerometersReading() : YarpTestCase("AccelerometersReading"),
robotName(""),
busType(BUSTYPE_UNKNOWN),
portName(""),
sampleTime(0.010)
{}

AccelerometersReading::~AccelerometersReading() { }

bool AccelerometersReading::setup(yarp::os::Property &configuration) {
    std::cout << "properties...\n" << configuration.toString() << "\n";
    // initialization goes here ...
    if(configuration.check("name"))
        setName(configuration.find("name").asString());

    // check input parameters
    RTF_ASSERT_ERROR_IF(configuration.check("robot"), "Missing 'robot' parameter");
    RTF_ASSERT_ERROR_IF(configuration.check("bus"),"Missing 'bus' parameter");
    RTF_ASSERT_ERROR_IF(configuration.check("part"),"Missing 'part' parameter");
    RTF_ASSERT_ERROR_IF(configuration.check("mtbList"),"Missing 'mtbList' parameter");

    // set class attributes accordingly
    // robot name
    this->robotName = configuration.find("robot").asString();
    // bus type
    std::string busTypeStr = configuration.find("bus").asString();
    if (!busTypeStr.compare("can")) {
        this->busType = BUSTYPE_CAN;
    }
    else if (!busTypeStr.compare("eth")) {
        this->busType = BUSTYPE_ETH;
    }
    else {
        this->busType = BUSTYPE_UNKNOWN;
    }
    // port name
    // (to be removed along with opening and connecting the port if wholeBodySensors API is used)
    std::string partName = configuration.find("part").asString();
    switch (this->busType) {
        case BUSTYPE_CAN:
            if (   partName.compare("torso")
                || partName.compare("left_arm")
                || partName.compare("right_arm"))
            {
                this->portName = "/" + this->robotName + "/" + partName + "_accelerometers/analog:o";
            }
            else
            {
                this->portName = "/" + this->robotName + "/" + partName + "_inertial/analog:o";
            }
            break;

        case BUSTYPE_ETH:
            this->portName = "/" + this->robotName + "/" + partName + "/inertialMTB";
            break;

        default:
            break;
    }

    // parse the MTB sensors list
    this->mtbListBottle = new Bottle(*configuration.find("mtbList").asList());

    // open the port
    RTF_ASSERT_ERROR_IF(port.open("/iCubTest/" + partName + "/inertialMTB:i"),
                        "opening port, is YARP network working?");

    // connect the port to our anonymous port
    RTF_TEST_REPORT(Asserter::format("connecting from %s output port to %s input port.\n",
                                     portName.c_str(), port.getName().c_str()));
    RTF_ASSERT_ERROR_IF(Network::connect(portName, port.getName()),
                        Asserter::format("could not connect to remote source port %s, MTB inertial sensor unavailable",
                                         portName.c_str()));
    RTF_TEST_REPORT(Asserter::format("Ready to read from MTB sensors:\n%s",this->mtbListBottle->toString().c_str()));

    return true;
}

void AccelerometersReading::tearDown() {
    // finalization goes here ...
    Network::disconnect(portName, port.getName());
    port.close();
}

void AccelerometersReading::run() {
    RTF_TEST_REPORT("Reading MTB accelerometers:");

    //Read data from sensors for about 5s
    for(int cycleIdx=0; cycleIdx<500; cycleIdx++)
    {
        ostringstream sizeErrorMessage, invalidMeasErrorMessage;

        Vector *readSensor = port.read();

        // check for reading failure
        RTF_TEST_FAIL_IF(readSensor, "could not read inertial data from sensor");

        // check size
        sizeErrorMessage
        << "sensor stream size issue: we should get "
        << this->mtbListBottle->size()
        << " sensor measurement vectors of 3 components!";
        RTF_TEST_FAIL_IF(readSensor->size() == 3*(this->mtbListBottle->size()), sizeErrorMessage.str());

        // look for invalid data
        for(int sensorIdx=0; sensorIdx<this->mtbListBottle->size(); sensorIdx++)
        {
            Vector sensorMeas = readSensor->subVector(sensorIdx*3, sensorIdx*3+2);
            invalidMeasErrorMessage
            << this->mtbListBottle->get(sensorIdx).asString()
            << " sensor measurement is invalid";
            RTF_TEST_FAIL_IF(sensorMeas(0) != -1.0
                             || sensorMeas(1) != -1.0
                             || sensorMeas(2) != -1.0, invalidMeasErrorMessage.str());
        }

        yarp::os::Time::delay(this->sampleTime);
    }
}

