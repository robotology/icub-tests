// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <cstdlib>
#include <sstream>

#include "MTBsensorParserEth.h"
#include "yarp/os/Bottle.h"
#include "yarp/sig/Vector.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

MTBsensorParserEth::MTBsensorParserEth() {}

MTBsensorParserEth::~MTBsensorParserEth() {}

bool MTBsensorParserEth::mapSensorData(yarp::sig::Vector *readSensor,
                                       std::vector<sensorTypeT> typeList, yarp::os::Bottle &sensorList,
                                       yarp::os::Bottle &availSensorList, std::string &errorMsg)
{
    /*
     * ==== MTB Data format ====:
     *
     * a = (n  6.0  (a1  b1 t1 x1 y1 x1)   .... (an  bn tn xn yn xn))
     * ai = pos of sensor ... see enum type
     * bi = accel (1) or gyro (2)
     * tn = time stamp.
     *
     */

    // Check the number of measurements
    if (2+6*(*readSensor)(0) != readSensor->size() || (*readSensor)(0) < sensorList.size()) {
        errorMsg = "Wrong total sensor data size!";
        return false;
    }

    // Check the format version number
    if ((*readSensor)(1) != this->version) {
        errorMsg = "Wrong format version!";
        return false;
    }

    // Resize lists
    this->reqMTBsensOffsets.resize(sensorList.size());

    // Build the mapping of the available sensors
    for(int readOff=2; readOff<readSensor->size(); readOff+=6)
    {
        int ai = int((*readSensor)(readOff));
        this->availMTBsensIDs[this->LUTmtbEnum2Id[ai]] = readOff;
        availSensorList.addString(this->LUTmtbEnum2Id[ai]);
    }

    // Check if required sensors (IDs and types) are available and build a matching index list
    for(int reqIdx=0; reqIdx<sensorList.size(); reqIdx++)
    {
        std::map<std::string,int>::iterator iter = this->availMTBsensIDs.find(sensorList.get(reqIdx).toString());
        if(iter != this->availMTBsensIDs.end()) // MTB sensor ID present in the list of availables
        {
            // retrieve respective offset
            int readOff = iter->second;
            // use index to check type...
            eOas_inertial_type_t bi = eOas_inertial_type_t((*readSensor)(readOff+1));
            if(eOas_inertial_type_t(typeList[reqIdx]) == bi)
            {
                // ...and save offset
                this->reqMTBsensOffsets[reqIdx] = readOff+3;
            }
            else
            {
                errorMsg = "Wrong MTB sensor type!";
                return false;
            }
        }
        else
        {
            errorMsg = "Required MTB sensor not available!";
            return false;
        }
    }

    // Save sensors RAW configuration for further quick check config is unchanged.
    this->rawSensorConfig.resize(2+2*(*readSensor)(0));
    this->rawSensorConfig.setSubvector(0,readSensor->subVector(0,1));
    for(int srcIdx=2, destIdx=2;
        srcIdx<readSensor->size();
        srcIdx+=6,destIdx+=2)
    {
        this->rawSensorConfig.setSubvector(destIdx,readSensor->subVector(srcIdx,srcIdx+1));
    }

    return true;
}

void MTBsensorParserEth::parseSensorMeas(yarp::sig::Vector * readSensor,
                                         std::vector< std::array<double,3> > &sensorMeasList)
{
    for(int sensorIdx=0; sensorIdx<this->reqMTBsensOffsets.size(); sensorIdx++)
    {
        sensorMeasList.resize(this->reqMTBsensOffsets.size());
        sensorMeasList[sensorIdx][0]=(*readSensor)(this->reqMTBsensOffsets[sensorIdx]+0);
        sensorMeasList[sensorIdx][1]=(*readSensor)(this->reqMTBsensOffsets[sensorIdx]+1);
        sensorMeasList[sensorIdx][2]=(*readSensor)(this->reqMTBsensOffsets[sensorIdx]+2);
    }
}

bool MTBsensorParserEth::checkControlData(yarp::sig::Vector * readSensor)
{
    // Get sensors configuration just read
    yarp::sig::Vector readSensConfig(this->rawSensorConfig.size());
    readSensConfig.setSubvector(0,readSensor->subVector(0,1));
    for(int srcIdx=2, destIdx=2;
        srcIdx<readSensor->size();
        srcIdx+=6,destIdx+=2)
    {
        readSensConfig.setSubvector(destIdx,readSensor->subVector(srcIdx,srcIdx+1));
    }

    // Compare last and current configurations
    return (this->rawSensorConfig == readSensConfig);
}

