// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

//#include <cstdlib>
//#include <sstream>
//#include <string>
//#include <vector>
//#include <array>
//
#include "Plotter.h"
#include "yarp/sig/Vector.h"

using namespace std;

Plotter::Plotter(){}

Plotter::~Plotter() {}

bool Plotter::setup(string plotName, string desc, string partName, yarp::os::Bottle& vars, string& statusMsg)
{
    // init parameters
    this->plotName = plotName; this->partName = partName; this->varLabels = vars;
    // open the ports
    if(!this->labelsPort.open("/iCubTest/yarpscope/" + this->plotName + "/" + this->partName + "/labels:o"))
    {
        statusMsg = "opening port, is YARP network working?";
        return false;
    }
    if(!this->rawDataPort.open("/iCubTest/yarpscope/" + this->plotName + "/" + this->partName + "/analog:o"))
    {
        statusMsg = "opening port, is YARP network working?";
        return false;
    }
    return true;

}

void Plotter::tear()
{
    this->labelsPort.close();
    this->rawDataPort.close();
}

void Plotter::plot(yarp::sig::Vector vec)
{
    // publish labels every 100 raw data instances
    this->labelsPublishCounter = (this->labelsPublishCounter+1)%100;
    if (this->labelsPublishCounter == 1) {
        this->labelsPort.prepare() = this->varLabels;
        this->labelsPort.write();
    }

    // publish raw data
    this->rawDataPort.prepare() = vec;
    this->rawDataPort.write();
}

