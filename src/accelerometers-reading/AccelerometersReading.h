// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _ACCELEROMETERSREADING_H_
#define _ACCELEROMETERSREADING_H_

#include <string>
#include <vector>
#include <fstream>

#include <rtf/yarp/YarpTestCase.h>
#include "IMTBsensorParser.h"
#include "IDataLoader.h"
#include "ValueDistribution.h"

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>

#define defaultReadingCycles 2000
#define expectedGravityNorm 9.8   // m/s^2
#define gravityNormInstTolerance 0.5  // m/s^2
#define gravityNormMeanTolerance 0.1  // m/s^2
#define gravityNormDevTolerance 1  // m/s^2
#define gravityAngleInstTolerance 20  // degrees
#define gravityAngleMeanTolerance 20  // degrees
#define gravityAngleDevTolerance 20  // degrees

namespace yarp {
    namespace sig {
        class Vector;
    }
    namespace os {
        class Property;
        class Bottle;
    }
}

enum busType_t {
    BUSTYPE_CAN,
    BUSTYPE_ETH,
    BUSTYPE_NTYPES,
    BUSTYPE_UNKNOWN
};


/**
 * \ingroup icub-tests
 * Check if inertial MTB sensor ports are correctly publishing accelerometer data, i.e. the expected accelerometers
 * re publishing data, and this data is valid (not 0xffff).
 * No further check on the content of the vector is done.
 *
 *  Accepts the following parameters:
 * | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
 * |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
 * | name           | string | -     | "AccelerometersReading" | No       | The name of the test. | -     |
 * | robot          | string | -     | -             | Yes      | The name of the robot.     | e.g. icub |
 * | bus            | string | -     | -             | Yes      | CAN or ETHERNET based robot. | - |
 * | part           | string | -     | -             | Yes      | The name of the robot part. | e.g. left_arm |
 * | mtbList        | string | -     | -             | Yes      | The sensors data mapping. | ordered list of sensor IDs |
 * | sampleTime     | double | s     | 0.010         | No       | The sample time of the control thread | |
 * | dataDumpFile   | string | -     | -             | Yes      | The file holding sensor data from the data dumper.| -  |
 * | subTests       | string | -     | "basic"       | No       | List of subtests to run. | -   |
 * | bins           | int    | -     | 50            | No       | List of subtests to run. | -   |
 *
 */
class AccelerometersReading : public YarpTestCase {
public:
    AccelerometersReading();
    virtual ~AccelerometersReading();

    virtual bool setup(yarp::os::Property& configuration);

    virtual void tearDown();

    virtual void run();

    /*
     * Getters
     */
    busType_t getBusType();

private:
    virtual bool setBusType(yarp::os::Property& configuration);

    virtual bool checkNparseSensors(std::vector<iDynTree::Vector3>& sensorMeasList);

    virtual bool checkDataConsistency(std::vector<iDynTree::Vector3>& sensorMeasList);

    virtual void bufferSensorData(std::vector<iDynTree::Vector3>& sensorMeasList);

    std::string robotName;
    busType_t busType;
    yarp::os::Bottle mtbList;
    yarp::os::Bottle subTests;
    std::vector<IMTBsensorParser::sensorTypeT> mtbTypeList;
    yarp::os::Bottle reordMtbList;
    const double sensorReadingCycles = defaultReadingCycles; // read MTB sensors for 20s
    IMTBsensorParser* sensorParserPtr;
    IDataLoader* dataLoader;
    std::vector< std::vector<iDynTree::Vector3> > sensorMeasMatList;
    std::vector<ValueDistribution> normDistribList;
//    std::vector<ValueDistribution> angleToGravityDistribList;
    int bins;
    
};

#endif //_ACCELEROMETERSREADING_H_
