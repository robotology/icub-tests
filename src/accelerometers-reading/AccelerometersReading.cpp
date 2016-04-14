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
#include "IMTBsensorParser.h"
#include "MTBsensorParserCan.h"
#include "MTBsensorParserEth.h"

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

    if(configuration.check("dataDumFile"))
    {
        setupFromLogFile(configuration);
    }
    else
    {
        setupFromYarpPort(configuration);
    }


    return true;
}

void AccelerometersReading::tearDown() {
    // finalization goes here ...
    Network::disconnect(this->portName, this->port.getName());
    this->port.close();
    delete(this->sensorParserPtr);
    this->dataDumpFileStr.close();
}

void AccelerometersReading::run() {
    std::string formatErrMsg;
    Bottle availSensorList;

    RTF_TEST_REPORT("Reading MTB accelerometers:");

    RTF_TEST_REPORT("Reading and parsing logged sensor data:");

    /*
     * Read data from log file and
     * compute the data mapping.
     */
//
//    for(int i=0; i<100; i++)
//    {
//        fs >> strFromFile;
//        std::cout << strFromFile.c_str();
//    }


    /*
     * Read data from sensors and
     * compute the data mapping.
     */
    Vector *readSensor = port.read();
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

        Vector *readSensor = port.read();

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

        yarp::os::Time::delay(this->sampleTime);
    }
}

/*
 * ===========================  LOCAL FUNCTIONS =====================================
 */

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

void AccelerometersReading::setupFromLogFile(yarp::os::Property& configuration)
{
    // open log file
    std::string logFileName = configuration.find("dataDumpFile").asString();
    this->dataDumpFileStr.open(logFileName.c_str(), std::fstream::out);
    
    RTF_ASSERT_ERROR_IF(this->dataDumpFileStr.is_open(),"Failed to open file");
}

void AccelerometersReading::setupFromYarpPort(yarp::os::Property& configuration)
{
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

    // sample time
    if(configuration.check("sampleTime"))
        this->sampleTime = configuration.find("sampleTime").asDouble();

    /*===============================================================================
     * Process the inputs
     *===============================================================================*/

    // open the port
    RTF_ASSERT_ERROR_IF(port.open("/iCubTest/" + partName + "/inertialMTB:i"),
                        "opening port, is YARP network working?");

    // connect the port to our anonymous port
    RTF_TEST_REPORT(Asserter::format("connecting from %s output port to %s input port.\n",
                                     this->portName.c_str(), this->port.getName().c_str()));
    RTF_ASSERT_ERROR_IF(Network::connect(this->portName, this->port.getName()),
                        Asserter::format("could not connect to remote source port %s, MTB inertial sensor unavailable",
                                         this->portName.c_str()));
    RTF_TEST_REPORT(Asserter::format("Ready to read from MTB sensors:\n%s",this->mtbList.toString().c_str()));
}


