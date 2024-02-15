#include <iostream>
#include <cmath>
#include <numeric>

#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>

#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/ResourceFinder.h>

#include "imu.h"

using namespace robottestingframework;
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(Imu)

Imu::Imu() : TestCase("Imu") { }

Imu::~Imu() { }

bool Imu::setup(yarp::os::Property& property) 
{
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("robot"), "The robot name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("port"), "The port name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("remoteControlBoards"), "Please, provide the controlboards name.");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("axesNames"), "Please, provide the controlled joints name.");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("model"), "Please, provide the urdf model path.");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("meanError"), "Please, provide the threshold error.");
    
    robotName = property.find("robot").asString(); // robot name
    portName = property.find("port").asString(); // name of the port from which the data are streamed
    errorMean = property.find("meanError").asFloat64(); //error mean
    modelName = property.find("model").asString(); // urdf model path
    yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string modelAbsolutePath = rf.findFileByName(modelName);
    
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Running IMU test on "+robotName+"...");
    
    yarp::os::Bottle *testList;
    testList = property.find("testList").asList();
    for(int test = 0; test < testList->size(); test++)
    {
        yarp::os::Bottle &sensors = property.findGroup(testList->get(test).asString());

        yarp::os::Bottle moveJoints;
        yarp::os::Bottle *inputMoveJoints;
        inputMoveJoints = sensors.find("moveJoints").asList();

        for (int movJoint = 0; movJoint < inputMoveJoints->size(); movJoint++)
        {
            moveJoints.addString(inputMoveJoints->get(movJoint).asString());
        }

        sensorMap[testList->get(test).asString()] = moveJoints;
        moveJoints.clear();
    }

    yarp::os::Property controlBoardOptions;
    controlBoardOptions.put("device", "remotecontrolboardremapper");

    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList(); 
    yarp::os::Bottle *inputControlBoards;
    inputControlBoards = property.find("remoteControlBoards").asList();
    for(int ctrlBoard = 0; ctrlBoard < inputControlBoards->size(); ctrlBoard++)
    {
        remoteControlBoardsList.addString("/"+robotName+"/"+inputControlBoards->get(ctrlBoard).asString());
    }

    yarp::os::Bottle axesNames;
    yarp::os::Bottle & axesList = axesNames.addList();
    yarp::os::Bottle *inputJoints;
    inputJoints = property.find("axesNames").asList();
    for(int ctrlJoint = 0; ctrlJoint < inputJoints->size(); ctrlJoint++)
    {
        axesList.addString(inputJoints->get(ctrlJoint).asString());
    }

    controlBoardOptions.put("remoteControlBoards", remoteControlBoards.get(0));
    controlBoardOptions.put("localPortPrefix", "/test");
    controlBoardOptions.put("axesNames", axesNames.get(0));

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(controlBoardDriver.open(controlBoardOptions), "Unable to open the controlBoard driver");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(controlBoardDriver.isValid(), "Device driver cannot be opened");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(controlBoardDriver.view(ipos), "Unable to open position control interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(controlBoardDriver.view(ienc), "Unable to open encoder interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(controlBoardDriver.view(iaxes), "Unable to open axes interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(controlBoardDriver.view(ilim), "Unable to open limits interface");

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getAxes(&axes), "Cannot get number of controlled axes");
    std::string axisName;
    
    for (int axIndex = 0; axIndex < axes; axIndex++)
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iaxes->getAxisName(axIndex, axisName), "Cannot get the name of controlled axes");
        axesVec.push_back(axisName);
    }
    
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(model.loadReducedModelFromFile(modelAbsolutePath.c_str(), axesVec), Asserter::format("Cannot load model from %s", modelAbsolutePath.c_str()));
    kinDynComp.loadRobotModel(model.model());

    yarp::os::Property MASclientOptions;
    MASclientOptions.put("device", "multipleanalogsensorsclient");
    MASclientOptions.put("remote", portName);
    MASclientOptions.put("local", "/imuTest"+portName);

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(MASclientDriver.open(MASclientOptions), "Unable to open the MAS client driver");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(MASclientDriver.isValid(), "Device driver cannot be opened");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(MASclientDriver.view(iorientation), "Unable to open orientation interface");
        
    outputPort.open("/test-imu/out");
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Opening port "+outputPort.getName()+"...");

    iDynTree::Vector3 baseLinkOrientationRad;
    baseLinkOrientationRad.zero();

    baseLinkOrientation = iDynTree::Rotation::RPY(baseLinkOrientationRad[0], baseLinkOrientationRad[1], baseLinkOrientationRad[2]);
    I_T_base = iDynTree::Transform(baseLinkOrientation, iDynTree::Position::Zero());
    iDynTree::Twist baseVelocity = iDynTree::Twist::Zero();

    unsigned int dofs = kinDynComp.model().getNrOfDOFs();
    s.resize(dofs);
    ds.resize(dofs);

    positions.resize(axes);
    velocities.resize(axes);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoders(positions.data()), "Cannot get joint positions");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoderSpeeds(velocities.data()), "Cannot get joint velocities");

    for(auto axIndex = 0; axIndex < axes; axIndex++)
    {
        s.setVal(axIndex, iDynTree::deg2rad(positions[axIndex]));
        ds.setVal(axIndex, iDynTree::deg2rad(velocities[axIndex]));
    }

    gravity.zero();
    gravity(2) = -9.81;

    kinDynComp.setRobotState(
        I_T_base,
        s,
        baseVelocity,
        ds,
        gravity
    );

    return true;
}

void Imu::tearDown() 
{
    outputPort.interrupt();
    outputPort.close();

    controlBoardDriver.close();
    MASclientDriver.close();
}

void Imu::run() 
{
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Starting reading IMU orientation values...");

    for (int sensorIndex = 0; sensorIndex < iorientation->getNrOfOrientationSensors(); sensorIndex++)
    {
        iorientation->getOrientationSensorName(sensorIndex, sensorName);

        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorMeasureAsRollPitchYaw(sensorIndex, rpyValues, timestamp), "Unable to obtain rpy measurements.");
        iorientation->getOrientationSensorFrameName(sensorIndex, frameName);
        iDynTree::Rotation I_R_FK = kinDynComp.getWorldTransform(frameName).getRotation(); 
        I_R_I_IMU = (I_R_FK * ((iDynTree::Rotation::RPY(iDynTree::deg2rad(rpyValues[0]), iDynTree::deg2rad(rpyValues[1]), iDynTree::deg2rad(rpyValues[2]))).inverse())); 
    
        double minLim;
        double maxLim;

        for (int movJoint = 0; movJoint < sensorMap[sensorName].size(); movJoint++)
        {
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ilim->getLimits(model.model().getJointIndex(sensorMap[sensorName].get(movJoint).toString()), &minLim, &maxLim), Asserter::format("Unable to get limits for joint %s", sensorMap[sensorName].get(movJoint).toString()));
            moveJoint(model.model().getJointIndex(sensorMap[sensorName].get(movJoint).toString()), minLim + 5, sensorIndex);
            yarp::os::Time::delay(1.);
            moveJoint(model.model().getJointIndex(sensorMap[sensorName].get(movJoint).toString()), maxLim - 5, sensorIndex);
            yarp::os::Time::delay(1.);
        }
    }
}

bool Imu::sendData(iDynTree::Vector3 expectedValues, iDynTree::Vector3 imuSignal)
{
    yarp::os::Bottle& out = outputPort.prepare();
    static yarp::os::Stamp stamp;

    stamp.update();
    outputPort.setEnvelope(stamp);
    out.clear();

    out.addFloat64(iDynTree::rad2deg(expectedValues[0]));
    out.addFloat64(iDynTree::rad2deg(expectedValues[1]));
    out.addFloat64(iDynTree::rad2deg(expectedValues[2]));

    out.addFloat64(iDynTree::rad2deg(imuSignal[0]));
    out.addFloat64(iDynTree::rad2deg(imuSignal[1]));
    out.addFloat64(iDynTree::rad2deg(imuSignal[2]));

    out.addFloat64(stamp.getTime());

    outputPort.writeStrict();    
    
    return true;
}

bool Imu::moveJoint(int ax, double pos, int sensorIndex)
{
    bool done = false;
    iDynTree::GeomVector3 error;
    std::vector<double> errorTot;
    iorientation->getOrientationSensorName(sensorIndex, sensorName);
    yarp::os::Time::delay(.1);

    ipos->positionMove(ax, pos);

    while(!done)
    {
        iorientation->getOrientationSensorMeasureAsRollPitchYaw(sensorIndex, rpyValues, timestamp);

        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoders(positions.data()), "Cannot get joint positions");
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoderSpeeds(velocities.data()), "Cannot get joint velocities");

        for (auto axIndex = 0; axIndex < axes; axIndex++)
        {
            s.setVal(axIndex, iDynTree::deg2rad(positions[axIndex]));
            ds.setVal(axIndex, iDynTree::deg2rad(velocities[axIndex]));
        }  

        kinDynComp.setRobotState(
        I_T_base,
        s,
        baseVelocity,
        ds,
        gravity);

        iDynTree::Rotation expectedImuSignal = kinDynComp.getWorldTransform(frameName).getRotation();
        iDynTree::Rotation imuSignal = (I_R_I_IMU * iDynTree::Rotation::RPY(iDynTree::deg2rad(rpyValues[0]), iDynTree::deg2rad(rpyValues[1]), iDynTree::deg2rad(rpyValues[2]))); 
        error = (expectedImuSignal * imuSignal.inverse()).log();

        errorTot.reserve(error.size());

        double sumOfSquares = std::accumulate(error.begin(), error.end(), 0.0,
        [](double accumulator, double element) {
            return accumulator + element * element;
        });
    
        double mag = std::sqrt(sumOfSquares);
        errorTot.push_back(mag);
 
        sendData(expectedImuSignal.asRPY(), imuSignal.asRPY());
        ipos->checkMotionDone(&done);
    }

    double maxError = *max_element(errorTot.begin(), errorTot.end());

    ROBOTTESTINGFRAMEWORK_TEST_CHECK(maxError < errorMean, Asserter::format("Testing %s for sensor %s: the max rotation angle error is %f rad!", axesVec[ax].c_str(), sensorName.c_str(), maxError));
    ipos->positionMove(ax, 0.0);

    return true;
}