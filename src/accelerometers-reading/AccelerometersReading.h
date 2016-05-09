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
#include "Plotter.h"

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/EigenHelpers.h>

#define defaultReadingCycles 2000
#define GravityNorm 9.80665   // m/s^2
#define GravNormInstTol 2  // m/s^2
#define GravNormMeanTol 0.2  // m/s^2
#define GravNormDevTol 0.3  // m/s^2
#define GravAngInstTol 30  // degrees
#define GravAngMeanTol 20  // degrees
#define GravAngDevTol 20  // degrees
#define accGain (2*GravityNorm/32768)
#define refAveragingWindowSize 100 // size of averaging window
#define relRefMeanTol 0.05 // tolerance for fixing a stable mean
// for building the ref for relative angle computations

namespace yarp {
    namespace sig {
        class Vector;
    }
    namespace os {
        class Property;
        class Bottle;
    }
}

class Plotter;

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
 * | bins           | int    | -     | 50            | No       | Number of bins in the meas. distributions. | -   |
 * | plot           | bool   | -     | false         | No       | plotting option. | -   |
 * | plotString     | string | -     | -             | Yes      | ...if 'plot' option present. | -   |
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
    class RelAngle {
    public:
        typedef enum
        {
            setRefAccVecForTheta = 0,
            setRefAccVecForPhi,
            refAccsFixed
        } refStatus_t;
        static const std::string refStatus2string[3];

        // Builder, destructor
        RelAngle();
        RelAngle(int sizeAveragingWindow);
        virtual ~RelAngle();

        // Update references for angle computation
        virtual refStatus_t updateRef(iDynTree::Vector3 vec);

        // Compute relative angle
        virtual void getRelAngle(iDynTree::Vector3 vec, double& phi, double& theta);

    private:
        typedef enum
        {
            fillBuffer = 0,
            computeStableMean
        } circBufferStatus_t;

        // Update mean with latest measurement and check for stable value
        virtual bool setMean(iDynTree::Vector3 vec, iDynTree::Vector3& mean);

        refStatus_t refStatus;
        circBufferStatus_t circBufferStatus;
        iDynTree::Vector3 refAccVecForTheta;
        iDynTree::Vector3 refAccVecForPhi;
        std::vector<iDynTree::Vector3> lastAccVec; // circular buffer
        iDynTree::Vector3 lastMean;
        int bufferIter;
        iDynTree::Vector3 lastAccVecCumul;
        double relPhi;
        double relTheta;
    };
        

    virtual bool setBusType(yarp::os::Property& configuration);

    virtual bool checkNparseSensors(std::vector<iDynTree::Vector3>& sensorMeasList);

    virtual bool checkDataConsistency(std::vector<iDynTree::Vector3>& sensorMeasList);

    virtual void bufferSensorData(std::vector<iDynTree::Vector3>& sensorMeasList);

    virtual void saveToFile(std::string filename, yarp::os::Bottle &b);

    std::string robotName;
    busType_t busType;
    yarp::os::Bottle mtbList;
    std::string part;
    yarp::os::Bottle subTests;
    int bins;
    bool plot;
    std::string plotString;

    std::vector<IMTBsensorParser::sensorTypeT> mtbTypeList;
    yarp::os::Bottle reordMtbList;
    const double sensorReadingCycles = defaultReadingCycles; // read MTB sensors for 20s
    IMTBsensorParser* sensorParserPtr;
    IDataLoader* dataLoader;
    std::vector< std::vector<iDynTree::Vector3> > sensorMeasMatList;
    std::vector<ValueDistribution> normDistribList;
//    std::vector<ValueDistribution> angleToGravityDistribList;
    std::vector<RelAngle> relAngleList;
    Plotter relGravPlotter;

};

#endif //_ACCELEROMETERSREADING_H_
