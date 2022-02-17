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
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/sig/Matrix.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


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
   
    IRemoteVariables* iVars{ nullptr };

    Property conf;
    conf.put("device", "remote_controlboard");
    conf.put("remote", "/icub/left_arm");
    conf.put("local", "/logger");
    if (!m_driver.open(conf)) {
        yError() << "Failed to open";
        return EXIT_FAILURE;
    }
    if (!(m_driver.view(iVars))) {
        m_driver.close();
        yError() << "Failed to view interfaces";
        return EXIT_FAILURE;
    }

    yarp::os::Bottle b,b1;

    iVars->getRemoteVariable("kinematic_mj", b);

    auto bb = b.get(1);

    int matrix_size = 4;
    string s;
    yarp::sig::Matrix matrix;

    matrix.resize(matrix_size,matrix_size);
    matrix.eye();
    int njoints [4];

    
    for(int i=0 ; i< b.size() ; i++)
    {
        Bottle bv;
        bv.fromString(b.get(i).toString());
        njoints[i] = sqrt(bv.size());

        yDebug() << " \n" << bv.toString();

       int ele = 0;
       if(i==0) {
           for (int r=0; r < njoints[i]; r++) 
            {
                for (int c=0; c < njoints[i]; c++) 
                {
                    matrix(r,c) = bv.get(ele).asFloat64();
                    ele++;
                }
            }

       }  
       else{
           for (int r=0; r < njoints[i]; r++) 
            {
                for (int c=0; c < njoints[i]; c++) 
                {
                    int jntprev = 0;
                    for (int j=0; j < i; j++) jntprev += njoints[j];
                    if(!jntprev > matrix_size)  matrix(r+jntprev,c+jntprev) = bv.get(ele).asFloat64();
                    ele++;
                }
            }
       }

      
    }

    yDebug() << " \n" << matrix.toString();

    m_driver.close();

    yInfo() << "Main returning...";
    return EXIT_SUCCESS;
    
}


