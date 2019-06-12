/*
 * iCub Robot Unit Tests (Robot Testing Framework)
 *
 * Copyright (C) 2015-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */


#include <math.h>
#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#include "TorqueControlGravityConsistency.h"

using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TorqueControlGravityConsistency)

TorqueControlGravityConsistency::TorqueControlGravityConsistency() : yarp::robottestingframework::TestCase("TorqueControlGravityConsistency"),
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
