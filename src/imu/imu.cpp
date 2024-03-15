#include <iostream>
#include <cmath>
#include <numeric>

#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>

#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/ResourceFinder.h>

#include <robometry/BufferConfig.h>
#include <robometry/BufferManager.h>

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
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("maxError"), "Please, provide the threshold error.");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("sensorsList"), "Please, provide the list of the sensors you want to check or 'all' if you want to test all the IMUs.");
    
    robotName = property.find("robot").asString(); // robot name
    portName = property.find("port").asString(); // name of the port from which the data are streamed
    errorMax = property.find("maxError").asFloat64(); //error mean
    modelName = property.find("model").asString(); // urdf model path
    yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string modelAbsolutePath = rf.findFileByName(modelName);
    
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Running IMU test on "+robotName+"...");

    yarp::os::Property controlBoardOptions;
    controlBoardOptions.put("device", "remotecontrolboardremapper");

    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList(); 
    yarp::os::Bottle *inputControlBoards;
    inputControlBoards = property.find("remoteControlBoards").asList();
    for(int ctrlBoard = 0; ctrlBoard < inputControlBoards->size(); ctrlBoard++)
    {
        partsList.push_back(inputControlBoards->get(ctrlBoard).asString());
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

    yarp::os::Bottle & sensorsNamesList = sensorsList.addList();
    yarp::os::Bottle *inputSensorsList;
    inputSensorsList = property.find("sensorsList").asList();

    if(inputSensorsList->size() == 0 || inputSensorsList->get(0).asString() == "all")
    {
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Testing all the IMUs available...");

        yarp::dev::IOrientationSensors* ior;
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(MASclientDriver.view(ior), "Unable to open the orientation interface");
        for(int sensorIndex = 0; sensorIndex < ior->getNrOfOrientationSensors(); sensorIndex++)
        {
            ior->getOrientationSensorName(sensorIndex, sensorName);
            sensorsNamesList.addString(sensorName);
        }    
    }

    else
    {
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Testing the list of provided IMUs...");
        for(int sensorIndex = 0; sensorIndex < inputSensorsList->size(); sensorIndex++)
        {
            sensorsNamesList.addString(inputSensorsList->get(sensorIndex).asString());
        }
    }

    yarp::os::Property MASremapperOptions;
    MASremapperOptions.put("device", "multipleanalogsensorsremapper");
    MASremapperOptions.put("OrientationSensorsNames", sensorsList.get(0));
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(MASremapperDriver.open(MASremapperOptions), "Unable to open the MAS remapper driver");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(MASremapperDriver.isValid(), "Device driver cannot be opened");

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(MASremapperDriver.view(imultiwrap), "Unable to open multiple wrapper interface");

    yarp::dev::PolyDriverList driverList;
    driverList.push(&MASclientDriver, "alljoints_inertials");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(imultiwrap->attachAll(driverList), "Unable to do the attach");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(MASremapperDriver.view(iorientation), "Unable to open orientation interface");
        
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

    setupRobometry();
    localBroker.resize(remoteControlBoards.get(0).asList()->size());

    return true;
}

void Imu::tearDown() 
{
    outputPort.interrupt();
    outputPort.close();

    controlBoardDriver.close();
    MASclientDriver.close();

    for(int i = 0; i < localBroker.size(); i++)
    {
        localBroker[i].stop();
    }
    
    scriptBroker.stop();
}

void Imu::run() 
{
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Starting reading IMU orientation values...");
    rpyValues.resize(sensorsList.get(0).asList()->size());

    for (int sensorIndex = 0; sensorIndex < sensorsList.get(0).asList()->size(); sensorIndex++)
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorName(sensorIndex, sensorName), "Unable to obtain rpy measurements.");
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorMeasureAsRollPitchYaw(sensorIndex, rpyValues[sensorIndex], timestamp), "Unable to obtain rpy measurements.");
        iorientation->getOrientationSensorFrameName(sensorIndex, frameName);
        iDynTree::Rotation I_R_FK = kinDynComp.getWorldTransform(frameName).getRotation(); 
        I_R_I_IMU = (I_R_FK * ((iDynTree::Rotation::RPY(iDynTree::deg2rad(rpyValues[sensorIndex][0]), iDynTree::deg2rad(rpyValues[sensorIndex][1]), iDynTree::deg2rad(rpyValues[sensorIndex][2]))).inverse()));    
    }

    setupBrokers();
    startMove();
}

bool Imu::startMove()
{
    bool done = false;
    iDynTree::GeomVector3 error;
    std::vector<std::vector<double>> errorTot(sensorsList.get(0).asList()->size());

    yarp::os::Time::delay(.1);

    while(!done)
    {
        for (int sensorIndex = 0; sensorIndex < sensorsList.get(0).asList()->size(); sensorIndex++)
        {
            sensorName = sensorsList.get(0).asList()->get(sensorIndex).asString();
            iorientation->getOrientationSensorMeasureAsRollPitchYaw(sensorIndex, rpyValues[sensorIndex], timestamp);
            iorientation->getOrientationSensorFrameName(sensorIndex, frameName);

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
            iDynTree::Rotation imuSignal = (I_R_I_IMU * iDynTree::Rotation::RPY(iDynTree::deg2rad(rpyValues[sensorIndex][0]), iDynTree::deg2rad(rpyValues[sensorIndex][1]), iDynTree::deg2rad(rpyValues[sensorIndex][2]))); 
            error = (expectedImuSignal * imuSignal.inverse()).log();

            bufferManager.push_back(positions, "joints_state::positions");
            bufferManager.push_back(velocities, "joints_state::velocities");
            bufferManager.push_back({expectedImuSignal.asRPY()[0], expectedImuSignal.asRPY()[1], expectedImuSignal.asRPY()[2]}, "orientations::" + sensorName + "::expected");
            bufferManager.push_back({imuSignal.asRPY()[0], imuSignal.asRPY()[1], imuSignal.asRPY()[2]}, "orientations::" + sensorName + "::measured");

            double sumOfSquares = std::accumulate(error.begin(), error.end(), 0.0,
            [](double accumulator, double element) {
                return accumulator + element * element;
            });

            double mag = std::sqrt(sumOfSquares);
            bufferManager.push_back(mag, "orientations::" + sensorName + "::error");

            errorTot[sensorIndex].push_back(mag);
        }

        ipos->checkMotionDone(&done);
    }

    for(int sensorIndex = 0; sensorIndex < sensorsList.get(0).asList()->size(); sensorIndex++)
    {
        sensorName = sensorsList.get(0).asList()->get(sensorIndex).asString();
        auto maxError = std::max_element(errorTot[sensorIndex].begin(), errorTot[sensorIndex].end());
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(*maxError < errorMax, Asserter::format("Testing sensor %s: the max rotation angle error is %f rad!", sensorName.c_str(), *maxError));
    }

    return true;
}

bool Imu::setupRobometry()
{
    robometry::BufferConfig bufferConfig;
    bufferConfig.auto_save = true;
    bufferConfig.yarp_robot_name = std::getenv("YARP_ROBOT_NAME");
    bufferConfig.filename = "test_imu";
    bufferConfig.file_indexing = "%Y_%m_%d_%H_%M_%S";
    bufferConfig.n_samples = 100000;
    
    bufferManager.addChannel({"joints_state::positions", {axesVec.size(), 1}, axesVec});
    bufferManager.addChannel({"joints_state::velocities", {axesVec.size(), 1}, axesVec});

    for(auto sensorIndex = 0; sensorIndex < sensorsList.get(0).asList()->size(); sensorIndex++) 
    {
        bufferManager.addChannel({"orientations::" + sensorsList.get(0).asList()->get(sensorIndex).asString() + "::expected", {3, 1}, {"r", "p", "y"}});
        bufferManager.addChannel({"orientations::" + sensorsList.get(0).asList()->get(sensorIndex).asString() + "::measured", {3, 1}, {"r", "p", "y"}});
        bufferManager.addChannel({"orientations::" + sensorsList.get(0).asList()->get(sensorIndex).asString() + "::error", {1, 1}, {"error"}});
    }
    
    return bufferManager.configure(bufferConfig);
}

void Imu::setupBrokers()
{
    strCmd = "ctpService";
    for(int i = 0; i < localBroker.size(); i++)
    {
        strParam = "--robot " + robotName + " --part " + partsList[i];
        localBroker[i].init(strCmd.c_str(), strParam.c_str(), nullptr, nullptr, nullptr, nullptr);
        localBroker[i].start();
        strParam.clear(); 
    }

    for (auto part : partsList)
    {
        if(part.find("leg") != std::string::npos)
            strParam = "";

        else
            strParam = "no_legs";
    }

    strCmd.clear();
    strCmd = "move.sh";

    scriptBroker.init(strCmd.c_str(), strParam.c_str(), nullptr, nullptr, nullptr, nullptr);
    scriptBroker.start();
}
