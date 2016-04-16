// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _MTBSENSORPARSERCAN_H_
#define _MTBSENSORPARSERCAN_H_

#include <string>
#include <vector>
#include "IMTBsensorParser.h"

/**
 * \ingroup icub-tests
 */

class MTBsensorParserCan : public IMTBsensorParser {
public:
    /**
     * Constructor
     *
     */
    MTBsensorParserCan();

    /**
     * Destructor
     *
     */
    virtual ~MTBsensorParserCan();

    /**
     * Checks the format and size of the sensor data read from a the YARP port,
     * matches the provided sensor list to the actual sensor IDs published on the YARP port,
     * computes the mapping between the sensor list and data offsets in the data stream.
     */
    virtual bool mapSensorData(yarp::sig::Vector *readSensor,
                               std::vector<sensorTypeT> typeList, yarp::os::Bottle &sensorList,
                               yarp::os::Bottle &availSensorList, std::string &errorMsg);

    /**
     * Returns the sensor data as a vector of measurements
     *
     */
    virtual void parseSensorMeas(yarp::sig::Vector * readSensor,
                                 std::vector< std::array<double,3> > &sensorMeasList);

    /**
     * Checks that control data didn't change
     *
     */
    virtual bool checkControlData(yarp::sig::Vector * readSensor);

private:
    int sensorListSize;

};
#endif //_MTBSENSORPARSERCAN_H_


