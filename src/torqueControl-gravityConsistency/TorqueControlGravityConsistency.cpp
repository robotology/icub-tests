// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <math.h>
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#include "TorqueControlGravityConsistency.h"

using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

// prepare the plugin
PREPARE_PLUGIN(TorqueControlGravityConsistency)

TorqueControlGravityConsistency::TorqueControlGravityConsistency() : yarp::rtf::TestCase("TorqueControlGravityConsistency"),
                                                                     yarpRobot(0)
{

}

TorqueControlGravityConsistency::~TorqueControlGravityConsistency()
{
}

bool TorqueControlGravityConsistency::setup(yarp::os::Property& property)
{
    yarp::os::ResourceFinder & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    yarp::os::Property yarpWbiConfiguration;
    std::string yarpWbiConfigurationFile;
    if(property.check("wbi_conf_file") && property.find("wbi_conf_file").isString())
    {
        yarpWbiConfigurationFile = rf.findFileByName(property.find("wbi_conf_file").asString());
    }
    else
    {
        yarpWbiConfigurationFile = rf.findFileByName("yarpWholeBodyInterface.ini");
    }

    //It may be convenient to overload some option of the configuration file,
    // so we load in the yarpWbiConfiguration also the option passed in the command line
    yarpWbiConfiguration.fromConfigFile(yarpWbiConfigurationFile);

    yarpWbiConfiguration.fromString(property.toString().c_str(),false);

    // Create yarpWholeBodyInterface
    std::string localName = "wbiTest";
    if( property.check("local") )
    {
        localName = property.find("local").asString();
    }

    wbi::wholeBodyInterface *yarpRobot = new yarpWbi::yarpWholeBodyInterface (localName.c_str(), yarpWbiConfiguration);

    wbi::IDList RobotMainJoints;
    std::string RobotMainJointsListName = "ROBOT_TORQUE_CONTROL_JOINTS";
    if( !yarpWbi::loadIdListFromConfig(RobotMainJointsListName,yarpWbiConfiguration,RobotMainJoints) )
    {
        fprintf(stderr, "[ERR] yarpWholeBodyInterface: impossible to load wbiId joint list with name %s\n",RobotMainJointsListName.c_str());
        return false;
    }

    yarpRobot->addJoints(RobotMainJoints);

    std::cout << "Joints added, calling init method" <<  std::endl;

    if(!yarpRobot->init())
    {
        std::cout << "Error: init() method failed" << std::endl;
        return false;
    }

    Time::delay(0.5);

    return true;
}

void TorqueControlGravityConsistency::tearDown()
{
    yarpRobot->close();
    delete yarpRobot;
    yarpRobot = 0;
}

void TorqueControlGravityConsistency::run()
{
    //Get the number of controlled degrees of freedom of the robot
    int dof = yarpRobot->getDoFs();
    printf("Number of (internal) controlled DoFs: %d\n", dof);

    //Allocate yarp vector of the right dimensions
    Vector q(dof), trqMeasured(dof), trqGravity(dof), generalizedBiasForces(dof+6), dq(dof);

    //Read position and torque estimates
    yarpRobot->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data());
    yarpRobot->getEstimates(wbi::ESTIMATE_JOINT_VEL, dq.data());
    yarpRobot->getEstimates(wbi::ESTIMATE_JOINT_TORQUE, trqMeasured.data());

    // Compute gravity compensation torques
    wbi::Frame world2base;
    world2base.identity();
    Vector baseTwist(6);
    baseTwist.zero();

    double m_gravity[3];
    m_gravity[0] = 0;
    m_gravity[1] = 0;
    m_gravity[2] = -9.81;

    yarpRobot->computeGeneralizedBiasForces(q.data(),world2base,dq.data(),baseTwist.data(),m_gravity,generalizedBiasForces.data());

    // We extract the joint torques from the generalized bias forces
    trqGravity = generalizedBiasForces.subVector(6,6+dof-1);

    for(size_t i=0; i < dof; i++)
    {
        wbi::ID wbiID;
        yarpRobot->getJointList().indexToID(i,wbiID);
        std::cerr << "Joint " << wbiID.toString() << " : " << std::endl;
        std::cerr << "\t Measured torque: " << trqMeasured[i] << "Nm" << std::endl;
        std::cerr << "\t Gravity  torque: " << trqGravity[i] << "Nm" << std::endl;
        std::cerr << "\t Difference     : " << trqMeasured[i]-trqGravity[i] << "Nm" << std::endl;
    }
}
