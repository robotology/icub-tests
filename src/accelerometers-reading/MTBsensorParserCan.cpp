// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <cstdlib>
#include <sstream>

#include "MTBsensorParserCan.h"
#include "yarp/os/Bottle.h"
#include "yarp/sig/Vector.h"

#include <iDynTree/Core/VectorFixSize.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

MTBsensorParserCan::MTBsensorParserCan() {}

MTBsensorParserCan::~MTBsensorParserCan() {}

bool MTBsensorParserCan::mapSensorData(yarp::sig::Vector *readSensor,
                                       std::vector<sensorTypeT> typeList, yarp::os::Bottle &sensorList,
                                       yarp::os::Bottle &availSensorList, std::string &errorMsg)
{
    // Check message size
    ostringstream streamErr;
    streamErr
    << "sensor stream size issue: we should get "
    << sensorList.size()
    << " sensor measurement vectors of 3 components!";

    if (readSensor->size() == 3*(sensorList.size()))
    {
        errorMsg = streamErr.str();
        return false;
    }
    else{
        // save size for future checks
        this->sensorListSize = sensorList.size();
        // The input list is supposed to already have the required order
        availSensorList = sensorList;
        return true;
    }
}

void MTBsensorParserCan::parseSensorMeas(yarp::sig::Vector * readSensor,
                                         std::vector<iDynTree::Vector3> &sensorMeasList)
{
    for(int sensorIdx=0,readIdx=0; sensorIdx<this->sensorListSize; sensorIdx++,readIdx+=3)
    {
        sensorMeasList.resize(this->sensorListSize);
        sensorMeasList[sensorIdx](0)=(*readSensor)(readIdx+0);
        sensorMeasList[sensorIdx](1)=(*readSensor)(readIdx+1);
        sensorMeasList[sensorIdx](2)=(*readSensor)(readIdx+2);
    }
}

bool MTBsensorParserCan::checkControlData(yarp::sig::Vector * readSensor)
{
    // Check message size
    return (readSensor->size() == 3*this->sensorListSize);
}

