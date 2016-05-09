// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _PLOTTER_H_
#define _PLOTTER_H_

#include <string>
#include "yarp/os/BufferedPort.h"

namespace yarp {
    namespace sig {
        class Vector;
    }
}


/**
 * \ingroup icub-tests
 * This class is not a YarpTestCase. It builds the context for plotting a YARP vector in yarpscope.
 * It opens the required ports and writes the data. It publishes the raw data in a port, and the 
 * variable names in another.
 *
 */

class Plotter {
public:
    /**
     * Constructors
     *
     */
    Plotter();

    /**
     * Destructor
     *
     */
    virtual ~Plotter();

    /**
     * Open/Close write port from where yarpscope will plot data
     */
    virtual bool setup(std::string plotName, std::string desc, std::string partName, yarp::os::Bottle& vars, std::string& statusMsg);
    virtual void tear();

    /**
     * Plot 1 scalar value
     */
    virtual void plot(yarp::sig::Vector vec);

    /**
     * Getters.
     */

private:
    std::string plotName; // this will be part of the destination port name
    std::string partName;
    yarp::os::BufferedPort<yarp::os::Bottle> labelsPort; // destination port for variable names
    yarp::os::BufferedPort<yarp::sig::Vector> rawDataPort; // destination port for raw data
    int labelsPublishCounter;
    yarp::os::Bottle varLabels;
};

#endif //_PLOTTER_H_

