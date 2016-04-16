// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <rtf/dll/Plugin.h>
#include <rtf/Asserter.h>
#include <array>

#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "AccelerometersReading.h"
#include "MTBsensorParserCan.h"
#include "MTBsensorParserEth.h"
#include "DataLoaderPort.h"
#include "DataLoaderFile.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;

// prepare the plugin
PREPARE_PLUGIN(AccelerometersReading)

AccelerometersReading::AccelerometersReading() : YarpTestCase("AccelerometersReading"),
robotName(""),
busType(BUSTYPE_UNKNOWN),
sensorParserPtr(NULL),
dataLoader(NULL)
{}

AccelerometersReading::~AccelerometersReading() { }

bool AccelerometersReading::setup(yarp::os::Property &configuration) {
    // Debug
    std::cout << "properties...\n" << configuration.toString() << "\n";
    // initialization goes here ...

    /*===============================================================================
     * Unconditional parameters
     *===============================================================================*/
    // std::vector<IMTBsensorParser::sensorTypeT> mtbTypeList;

    // check input parameters
    RTF_ASSERT_ERROR_IF(configuration.check("robot"), "Missing 'robot' parameter");
    RTF_ASSERT_ERROR_IF(configuration.check("bus"),"Missing 'bus' parameter");
    RTF_ASSERT_ERROR_IF(configuration.check("mtbList"),"Missing 'mtbList' parameter");

    // set class attributes accordingly

    // Test name
    if(configuration.check("name"))
        setName(configuration.find("name").asString());

    // robot name
    this->robotName = configuration.find("robot").asString();

    // bus type and respective sensor parser
    RTF_ASSERT_ERROR_IF(setBusType(configuration),"Robot bus type unknown");

    // parse the MTB sensors list
    this->mtbList = *configuration.find("mtbList").asList();

    // For now, sensor type will always be Accelerometers
    this->mtbTypeList.resize(this->mtbList.size());
    this->mtbTypeList.assign(this->mtbTypeList.size(), IMTBsensorParser::SENSORTYPE_ACC);

    /*===============================================================================
     * Conditional parameters (dump file or robot connection info)
     *===============================================================================*/

    // check input parameters
    RTF_ASSERT_ERROR_IF(configuration.check("dataDumpFile")
                        || configuration.check("part"),
                        "Missing 'part' and 'dataDumpFile' parameter. You should provide at least one of them");

    // select data loader (which will load data from a YARP port or a file)
    if(configuration.check("dataDumpFile"))
    {
        this->dataLoader = new DataLoaderFile();
    }
    else
    {
        this->dataLoader = new DataLoaderPort(*this);
    }

    // Create the access to the sensor data (open the YARP port or file)
    std::string statusMsg;
    RTF_ASSERT_ERROR_IF(this->dataLoader->setup(configuration, statusMsg), statusMsg.c_str());
    // connection succeded. Print message from data loader
    RTF_TEST_REPORT(statusMsg.c_str());

    RTF_TEST_REPORT(Asserter::format("Ready to read data from MTB sensors:\n%s",this->mtbList.toString().c_str()));

    return true;
}

void AccelerometersReading::tearDown() {
    // finalization goes here ...
    if(this->sensorParserPtr) {delete(this->sensorParserPtr);}
    if(this->dataLoader)
    {
        this->dataLoader->tearDown();
        delete(this->dataLoader);
    }
}

void AccelerometersReading::run() {
    std::string formatErrMsg;
    Bottle availSensorList;

    RTF_TEST_REPORT("Reading data from the MTB accelerometers:");

    /*
     * Read data from sensors and
     * compute the data mapping.
     */
    Vector *readSensor = this->dataLoader->read();
    // check for reading failure
    RTF_TEST_FAIL_IF(readSensor, "could not read inertial data from sensor");

    // Check format and size. Build the sensor data mapping and save the control
    // data (sensor IDs, types, ...)
    RTF_TEST_FAIL_IF(this->sensorParserPtr->mapSensorData(readSensor,
                                                          this->mtbTypeList, this->mtbList,
                                                          availSensorList, formatErrMsg), formatErrMsg);

    /*
     * Read data from sensors for about 5s
     */
    for(int cycleIdx=0; cycleIdx<this->sensoReadingCycles; cycleIdx++)
    {
        // error messages and sensor data buffer allocation
        ostringstream invalidMeasErrMsg;
        std::vector< std::array<double,3> > sensorMeasList;

        Vector *readSensor = this->dataLoader->read();

        // check for reading failure
        RTF_TEST_FAIL_IF(readSensor, "could not read inertial data from sensor");

        // Check that control data didn't change
        RTF_TEST_FAIL_IF(this->sensorParserPtr->checkControlData(readSensor), "Message configuration changed");

        // Get data from sensors
        this->sensorParserPtr->parseSensorMeas(readSensor, sensorMeasList);

        // look for invalid data
        for(int sensorIdx=0; sensorIdx<this->mtbList.size(); sensorIdx++)
        {
            std::array<double,3> sensorMeas = sensorMeasList[sensorIdx];
            invalidMeasErrMsg
            << this->mtbList.get(sensorIdx).toString()
            << " sensor measurement is invalid";

            RTF_TEST_FAIL_IF(sensorMeas[0] != -1.0
                             || sensorMeas[1] != -1.0
                             || sensorMeas[2] != -1.0, invalidMeasErrMsg.str());
        }

        this->dataLoader->delayBeforeRead();
    }
}

/*
 * ===========================  LOCAL FUNCTIONS =====================================
 */

busType_t AccelerometersReading::getBusType()
{
    return this->busType;
}

bool AccelerometersReading::setBusType(yarp::os::Property &configuration)
{
    std::string busTypeStr = configuration.find("bus").asString();
    if (!busTypeStr.compare("can")) {
        this->busType = BUSTYPE_CAN;
        this->sensorParserPtr = new MTBsensorParserCan();
        return true;
    }
    else if (!busTypeStr.compare("eth")) {
        this->busType = BUSTYPE_ETH;
        this->sensorParserPtr = new MTBsensorParserEth();
        return true;
    }
    else {
        this->busType = BUSTYPE_UNKNOWN;
        return false;
    }
}

