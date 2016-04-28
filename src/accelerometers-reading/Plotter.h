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
 * This class is not a YarpTestCase. It bilds the context for plotting a YARP vector in yarpscope
 * (open port, write YARP vector).
 *
 */

class Plotter {
public:
    /**
     * Constructor
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
    virtual bool setup(std::string partName, std::string var, std::string statusMsg);
    virtual void tear();

    /**
     * Plot 1 scalar value
     */
    virtual void plot(double value);

    /**
     * Getters.
     */

private:
    std::string portName; // sink port we will write to
    yarp::os::BufferedPort<yarp::sig::Vector> port; // anonymous destination port
};

#endif //_PLOTTER_H_

