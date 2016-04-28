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

Plotter::Plotter() {}

Plotter::~Plotter() {}

bool Plotter::setup(string partName, string var, string statusMsg)
{
    // open the port
    if(!this->port.open("/iCubTest/" + partName + "/yarpscope/" + var + ":o"))
    {
        statusMsg = "opening port, is YARP network working?";
        return false;
    }
    return true;

}

void Plotter::tear()
{
    this->port.close();
}

void Plotter::plot(double value)
{

}

