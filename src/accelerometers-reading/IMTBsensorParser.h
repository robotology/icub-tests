// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _IMTBSENSORPARSER_H_
#define _IMTBSENSORPARSER_H_

#include <string>
#include <vector>
#include <array>

#include <iDynTree/Core/VectorFixSize.h>

namespace yarp {
    namespace os {
        class Bottle;
    }
    namespace sig {
        class Vector;
    }
}


/**
 * \ingroup icub-tests
 * Checks the format and size of the sensor data read from a the YARP port.
 * Matches the provided sensor list to the actual sensor IDs published on the YARP port.
 * Computes the mapping between the sensor list and data offsets in the data stream.
 * Returns the sensor data as a vector of measurements.
 *
 */

class IMTBsensorParser {
public:
    enum sensorTypeT {
        SENSORTYPE_NONE=0,
        SENSORTYPE_ACC,
        SENSORTYPE_GYRO
    };

    /**
     * Destructor
     *
     */
    virtual ~IMTBsensorParser() = 0;

    /**
     * Checks the format and size of the sensor data read from a the YARP port,
     * matches the provided sensor list to the actual sensor IDs published on the YARP port,
     * computes the mapping between the sensor list and data offsets in the data stream.
     */
    virtual bool mapSensorData(yarp::sig::Vector *readSensor,
                               std::vector<sensorTypeT> typeList, yarp::os::Bottle &sensorList,
                               yarp::os::Bottle &availSensorList, std::string &errorMsg) = 0;

    /**
     * Returns the sensor data as a vector of measurements
     *
     */
    virtual void parseSensorMeas(yarp::sig::Vector * readSensor,
                                 std::vector<iDynTree::Vector3> &sensorMeasList) = 0;

    /**
     * Checks that control data didn't change
     *
     */
    virtual bool checkControlData(yarp::sig::Vector * readSensor) = 0;

};
#endif //_IMTBSENSORPARSER_H_


