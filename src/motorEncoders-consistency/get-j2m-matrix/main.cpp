/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <cstdlib>
#include <cmath>
#include <limits>
#include <vector>
#include <fstream>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IRemoteVariables.h>
#include<iostream>
#include<vector>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

struct DataExperiment {
    double t;
    double pid_ref;
    double pid_out;
    double enc;
};



int main(int argc, char * argv[])
{
    Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "Unable to find YARP server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);

    auto joint_id = rf.check("joint-id", Value(0)).asInt();
    auto set_point = rf.check("set-point", Value(10.)).asDouble();
    auto cycles = rf.check("cycles", Value(1)).asInt();
    auto T = rf.check("T", Value(2.)).asDouble();

    PolyDriver m_driver;
    IPositionControl* iPos{ nullptr };
    IPidControl* iPid{ nullptr };
    IControlMode* iCm{ nullptr };
    IEncoders* iEnc{ nullptr };
    IRemoteVariables* iVars{ nullptr };

    std::ofstream file;

    file.open("output.log");

    std::vector<DataExperiment> data_vec;
    data_vec.reserve(1000);

    Property conf;
    conf.put("device", "remote_controlboard");
    conf.put("remote", "/icub/left_leg");
    conf.put("local", "/logger");
    if (!m_driver.open(conf)) {
        yError() << "Failed to open";
        return EXIT_FAILURE;
    }
    if (!(m_driver.view(iPos) && m_driver.view(iPid) &&
          m_driver.view(iCm) && m_driver.view(iEnc)  && m_driver.view(iVars))) {
        m_driver.close();
        yError() << "Failed to view interfaces";
        return EXIT_FAILURE;
    }

    yarp::os::Bottle b,b1;


    iVars->getRemoteVariablesList(&b1);
    yInfo() << "list variables : " << b1.toString();

    iVars->getRemoteVariable("kinematic_j2m", b);
    // iVars->getRemoteVariable("gearbox_M2J", b1);

    yInfo() << "kinematic_j2m : " << b.toString() << " lenght : " << b.size();
    auto bb = b.get(1);

    int matrix_size = 6;
    string s;

    for(int i=0 ; i< b.size() ; i++)
    {
        s += " " + b.get(i).toString();

    }

    Bottle bv;
    bv.fromString(s);

    yDebug() << bv.toString();

    yDebug() << bv.size();

    yDebug() << bv.get(0).asFloat64();

 
   
    
    m_driver.close();
    file.close();

    yInfo()<<"Main returning...";
    return EXIT_SUCCESS;
    
}


