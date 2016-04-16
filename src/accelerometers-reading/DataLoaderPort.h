// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _DATALOADERPORT_H_
#define _DATALOADERPORT_H_

#include "IDataLoader.h"
#include "yarp/os/BufferedPort.h"

namespace yarp {
    namespace os {
        class Property;
    }
    namespace sig {
        class Vector;
    }
}

class AccelerometersReading;

/**
 * \ingroup icub-tests
 * This class is not a YarpTestCase. It opens the YARP port for reading the sensor data.
 * It also handles the destruction of associated objects or the closure of opened files.
 *
 */

class DataLoaderPort : public IDataLoader {
public:
    /**
     * Constructor
     *
     */
    DataLoaderPort(AccelerometersReading &test);

    /**
     * Destructor
     *
     */
    virtual ~DataLoaderPort();

    /**
     * Opens the YARP port for reading the sensor data.
     */
    virtual bool setup(yarp::os::Property& configuration, std::string &errorMsg);

    /**
     * It also handles the destruction of associated objects
     * or the closure of opened files.
     */
    virtual void tearDown();

    /**
     * Reads one vector from the YARP port.
     *
     */
    virtual yarp::sig::Vector* read();

    /**
     * Runs a delay between two consecutive data readouts
     */
    virtual void delayBeforeRead();

private:
    AccelerometersReading& invokingTest; // test calling this data loader
    std::string portName; // source port we will read from
    yarp::os::BufferedPort<yarp::sig::Vector> port; // anonymous destination port
    double sampleTime; // saplimg period of sensor measurements and publication on the yarp port

};
#endif //_DATALOADERPORT_H_


