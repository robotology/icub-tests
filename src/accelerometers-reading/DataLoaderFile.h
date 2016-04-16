// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _DATALOADERFILE_H_
#define _DATALOADERFILE_H_

#include <fstream>

#include "IDataLoader.h"
#include <yarp/sig/Vector.h>

namespace yarp {
    namespace os {
        class Property;
    }
    namespace sig {
        class Vector;
    }
}

/**
 * \ingroup icub-tests
 * This class is not a YarpTestCase. It opens the dump file for reading the sensor data.
 * It also handles the destruction of associated objects or the closure of opened files.
 *
 */

class DataLoaderFile : public IDataLoader {
public:
    /**
     * Constructor
     *
     */
    DataLoaderFile();

    /**
     * Destructor
     *
     */
    virtual ~DataLoaderFile();

    /**
     * Opens the dump file for reading the sensor data.
     */
    virtual bool setup(yarp::os::Property& configuration, std::string &errorMsg);

    /**
     * It also handle the destruction of associated objects
     * or the closure of opened files.
     */
    virtual void tearDown();

    /**
     * Reads one line of the dump file.
     *
     */
    virtual yarp::sig::Vector* read();

    /**
     * Runs a delay between two consecutive data readouts
     */
    virtual void delayBeforeRead();

private:
    std::ifstream dataDumpFileStr; // Dump file descriptor
    yarp::sig::Vector yarpVecFromFile; // YARP vector for reading one line from the file
};
#endif //_DATALOADERFILE_H_


