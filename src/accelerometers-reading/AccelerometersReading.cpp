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
#include <math.h>

#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "AccelerometersReading.h"
#include "MTBsensorParserCan.h"
#include "MTBsensorParserEth.h"
#include "DataLoaderPort.h"
#include "DataLoaderFile.h"
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/EigenHelpers.h>
//#include "LinearMotionVector3.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;

typedef Eigen::Matrix<double,3,1> EigenVector3d;

static double norm(iDynTree::Vector3& vector);
static double angleV1toV2(iDynTree::Vector3& vector1, iDynTree::Vector3& vector2);

// prepare the plugin
PREPARE_PLUGIN(AccelerometersReading)

AccelerometersReading::AccelerometersReading() : YarpTestCase("AccelerometersReading"),
robotName(""),
busType(BUSTYPE_UNKNOWN),
bins(50),
plot(false),
sensorParserPtr(NULL),
dataLoader(NULL)
{}

AccelerometersReading::~AccelerometersReading() {}

bool AccelerometersReading::setup(yarp::os::Property &configuration) {
    // Debug
    RTF_TEST_REPORT(Asserter::format("properties...\n%s\n",configuration.toString().c_str()));
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
    this->sensorMeasMatList.resize(this->mtbList.size());

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

    // check requested sub tests
    if (configuration.check("subTests")) {
        this->subTests = *configuration.find("subTests").asList();
    }

    // Get the number of bins for the norm and angle distributions
    if (configuration.check("bins")) {
        this->bins = configuration.find("bins").asInt();
    }
/*
    // Get plotting option and plot command
    if (configuration.check("plot")) {
        this->plot = configuration.find("plot").asBool();
        RTF_ASSERT_ERROR_IF(configuration.check("plotString"), "Missing 'plotString' parameter");
        this->plotString = configuration.find("plotString").asString();
    }
*/
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
     * Create distributions
     */
    this->normDistribList.resize(this->mtbList.size());
    for(int sensorIdx=0; sensorIdx<this->mtbList.size(); sensorIdx++)
    {
        ValueDistribution acc_i_measDistrib(this->sensorReadingCycles,9,11,this->bins);
        this->normDistribList[sensorIdx] = acc_i_measDistrib;
    }

    /*
     * Read data from sensors for about 200s
     */
    for(int cycleIdx=0; cycleIdx<this->sensorReadingCycles; cycleIdx++)
    {
        // Check there is no reading failure and no invalid data (0xFFFF...)
        //and return parsed sensor data.
        vector<iDynTree::Vector3> sensorMeasList;
        if(!this->checkNparseSensors(sensorMeasList)) {break;}

        // Check norm and angle w.r.t. expected gravity theoretical vector
        if(!this->checkDataConsistency(sensorMeasList)) {break;}

        // save parsed and computed data
        this->bufferSensorData(sensorMeasList);

        // Delay before next port or file line read
        this->dataLoader->delayBeforeRead();
    }

    /*
     * Evaluate distributions
     */
    for(int sensorIdx=0; sensorIdx<this->mtbList.size(); sensorIdx++)
    {
        this->normDistribList[sensorIdx].evalDistrParams();
        ValueDistribution::distr_t distrParams = this->normDistribList[sensorIdx].getDistr();
        RTF_TEST_REPORT(Asserter::format("\nAccelerometer %s :\n"
                                         "gravity norm \t mean \t\t standard deviation \t\t min \t\t max\n"
                                         "\t\t %g \t\t %g \t\t %g \t\t %g\n"
                                         "gravity angle \t mean \t\t standard deviation \t\t min \t\t max\n"
                                         "\t\t %g \t\t %g \t\t %g \t\t %g\n",
                                         this->mtbList.get(sensorIdx).toString().c_str(),
                                         distrParams.mean,
                                         distrParams.sigma,
                                         distrParams.min,
                                         distrParams.max,
                                         0.0,0.0,0.0,0.0));
        RTF_TEST_FAIL_IF(abs(distrParams.mean-GravityNorm)<GravNormMeanTol,
                         Asserter::format("Average norm beyond tolerance of %f",GravNormMeanTol));
        RTF_TEST_FAIL_IF(distrParams.sigma<GravNormDevTol,
                         Asserter::format("Standard deviation of norm beyond tolerance of %f",GravNormDevTol));
/*
        // Plot distribution
        Bottle histToFile;
        for(int idx=0; idx<distrParams.hist.size(); idx++) {histToFile.add(distrParams.hist[idx]);}
        saveToFile("plotAcc" + this->mtbList.get(sensorIdx).toString() + ".dat", histToFile);
        if(this->plot)
        {
            system(this->plotString.c_str());
        }
        else
        {
            RTF_TEST_REPORT("Test is finished. Please check if collected date are ok, by using following command: ");
            RTF_TEST_REPORT(RTF::Asserter::format("%s", this->plotString.c_str()));
        }
 */
    }
    std::cout<<"\n\n";
}

busType_t AccelerometersReading::getBusType()
{
    return this->busType;
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

bool AccelerometersReading::checkNparseSensors(std::vector<iDynTree::Vector3>& sensorMeasList)
{
    ostringstream invalidMeasErrMsg;

    Vector *readSensor = this->dataLoader->read();

    // check for reading failure
    RTF_TEST_FAIL_IF(readSensor, "could not read inertial data from sensor");

    // Check that control data didn't change
    RTF_TEST_FAIL_IF(this->sensorParserPtr->checkControlData(readSensor), "Message configuration changed");

    // Get data from sensors
    this->sensorParserPtr->parseSensorMeas(readSensor, sensorMeasList);

    // look for invalid data and convert raw data
    for(int sensorIdx=0; sensorIdx<sensorMeasList.size(); sensorIdx++)
    {
        iDynTree::Vector3 sensorMeas = sensorMeasList[sensorIdx];
        invalidMeasErrMsg
        << this->mtbList.get(sensorIdx).toString()
        << " sensor measurement is invalid";

        RTF_TEST_FAIL_IF(sensorMeas(0) != -1.0
                         || sensorMeas(1) != -1.0
                         || sensorMeas(2) != -1.0, invalidMeasErrMsg.str());
        cout << "Sensor " << this->mtbList.get(sensorIdx).toString() << " : " << sensorMeas.toString() << "  |  ";

        // convert from raw to m/s^2 acceleration
        iDynTree::toEigen(sensorMeasList[sensorIdx]) = iDynTree::toEigen(sensorMeas)*accGain;
    }

    cout << endl;
    return true;
}

bool AccelerometersReading::checkDataConsistency(std::vector<iDynTree::Vector3>& sensorMeasList)
{
    bool status;

    for (int sensorIdx=0; sensorIdx<sensorMeasList.size(); sensorIdx++)
    {
        // compute norm, check limits and add to distribution
        double measGravityNorm = norm(sensorMeasList[sensorIdx]);
        RTF_TEST_FAIL_IF(status = (abs(measGravityNorm-GravityNorm)<GravNormInstTol),
                         Asserter::format("Measured gravity norm is out of limits (%f).",measGravityNorm));

        this->normDistribList[sensorIdx].add(measGravityNorm);

/*      // plot gravity norm

        // compute angle, check limits w.r.t. gravity and add to distribution
        double measGravityAngle = angleV1toV2(sensorMeasList[sensorIdx], grav0);
        RTF_TEST_FAIL_IF(measGravityAngle>gravityAngleTolerance,
                         "Measured gravity angle is out of limits");*/
    }

    return status;
}

void AccelerometersReading::bufferSensorData(std::vector<iDynTree::Vector3>& sensorMeasList)
{
    // save each sensor value into the respective measurements matrix;
    for (int sensorIdx=0; sensorIdx<sensorMeasList.size(); sensorIdx++)
    {
        this->sensorMeasMatList[sensorIdx].push_back(sensorMeasList[sensorIdx]);
    }
}

void AccelerometersReading::saveToFile(std::string filename, yarp::os::Bottle &b)
{
    std::fstream fs;
    fs.open (filename.c_str(), std::fstream::out);

    std::string s = b.toString();
    fs << s << endl;

    fs.close();
}

static double norm(iDynTree::Vector3& vector)
{
    return sqrt(iDynTree::toEigen(vector).transpose()*iDynTree::toEigen(vector));
}

static double angleV1toV2(iDynTree::Vector3& vector1, iDynTree::Vector3& vector2)
{
    return 0;
}


