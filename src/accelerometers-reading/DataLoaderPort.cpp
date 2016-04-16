// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

//#include <cstdlib>
//#include <sstream>
//#include <string>
//#include <vector>
//#include <array>
//
#include "DataLoaderPort.h"
#include "yarp/os/Property.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/Time.h"
#include <yarp/os/BufferedPort.h>
#include "AccelerometersReading.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;

DataLoaderPort::DataLoaderPort(AccelerometersReading &test) :
invokingTest(test),
portName(""),
sampleTime(0.010)
{}

DataLoaderPort::~DataLoaderPort() {}

bool DataLoaderPort::setup(yarp::os::Property& configuration, std::string &statusMsg)
{
    // port name
    string partName = configuration.find("part").asString();
    string robotName = configuration.find("robot").asString();

    switch (this->invokingTest.getBusType()) {
        case BUSTYPE_CAN:
            if (   partName.compare("torso")
                || partName.compare("left_arm")
                || partName.compare("right_arm"))
            {
                this->portName = "/" + robotName + "/" + partName + "_accelerometers/analog:o";
            }
            else
            {
                this->portName = "/" + robotName + "/" + partName + "_inertial/analog:o";
            }
            break;

        case BUSTYPE_ETH:
            this->portName = "/" + robotName + "/" + partName + "/inertialMTB";
            break;

        default:
            break;
    }

    // sample time
    if(configuration.check("sampleTime"))
        this->sampleTime = configuration.find("sampleTime").asDouble();

    /*===============================================================================
     * Process the inputs
     *===============================================================================*/

    // open the port
    if(!this->port.open("/iCubTest/" + partName + "/inertialMTB:i"))
    {
        statusMsg = "opening port, is YARP network working?";
        return false;
    }

    // connect the port to our anonymous port
    if(!Network::connect(this->portName, this->port.getName()))
    {
        statusMsg = "could not connect to remote source port " + this->portName
        + ", MTB inertial sensor unavailable";
        return false;
    }

    // Connection succeeded
    statusMsg = "connected from " + this->portName
    + " output port to " + this->port.getName() + " input port.\n";

    return true;
}

void DataLoaderPort::tearDown()
{
    Network::disconnect(this->portName, this->port.getName());
    this->port.close();
}

yarp::sig::Vector* DataLoaderPort::read()
{
    return this->port.read();
}

void DataLoaderPort::delayBeforeRead()
{
    yarp::os::Time::delay(this->sampleTime);
}

