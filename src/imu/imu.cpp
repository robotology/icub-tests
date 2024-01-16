#include <iostream>
#include <cmath>
#include <numeric>

#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>

#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>

#include "imu.h"

using namespace robottestingframework;
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(Imu)

Imu::Imu() : TestCase("Imu") { }

Imu::~Imu() { }

bool Imu::setup(yarp::os::Property& property) {
    
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("robot"), "The robot name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("part"), "The part name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("model"), "Please, provide the urdf model path.");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("frame"), "Please, provide the frame name.");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("sensor"), "Please, provide the sensor name.");
    
    robotName = property.find("robot").asString(); // robot name
    partName = property.find("part").asString(); // part of the robot on which the sensor is mountekd
    modelPath = property.find("model").asString(); // urdf model path
    frameName = property.find("frame").asString(); // frame on which the sensor is attached
    sensorName = property.find("sensor").asString(); // sensor name within urdf

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Running IMU test on "+robotName+" for "+partName);

    yarp::os::Property options1;
    options1.put("device", "multipleanalogsensorsclient");
    options1.put("remote", "/"+robotName+"/"+partName+"/inertials");
    options1.put("local", "/imuTest/"+robotName+"/"+partName);
    
    driver1 = new yarp::dev::PolyDriver(options1);
    
    yarp::os::Property options2;
    options2.put("device", "remotecontrolboardremapper");
    yarp::os::Bottle axesNames;
    yarp::os::Bottle & axesList = axesNames.addList();
    axesList.addString("neck_pitch");
    axesList.addString("neck_roll");
    axesList.addString("neck_yaw");
    axesList.addString("torso_pitch");
    axesList.addString("torso_roll");
    axesList.addString("torso_yaw");
    axesList.addString("l_shoulder_pitch");
    axesList.addString("l_shoulder_roll");
    axesList.addString("l_shoulder_yaw");
    axesList.addString("l_elbow");
    axesList.addString("r_shoulder_pitch");
    axesList.addString("r_shoulder_roll");
    axesList.addString("r_shoulder_yaw");
    axesList.addString("r_elbow");

    options2.put("axesNames", axesNames.get(0));

    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
    remoteControlBoardsList.addString("/"+robotName+"/torso");
    remoteControlBoardsList.addString("/"+robotName+"/head");
    remoteControlBoardsList.addString("/"+robotName+"/left_arm");
    remoteControlBoardsList.addString("/"+robotName+"/right_arm");
    options2.put("remoteControlBoards", remoteControlBoards.get(0));
    options2.put("localPortPrefix", "/test");

    driver2 = new yarp::dev::PolyDriver(options2);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver1->isValid(), "Device driver cannot be opened");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver1->view(iorientation), "Unable to open orientation interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver2->isValid(), "Device driver cannot be opened");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver2->view(ipos), "Unable to open position control interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver2->view(ienc), "Unable to open encoder interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver2->view(iaxes), "Unable to open axes interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver2->view(ilim), "Unable to open limits interface");


    outputPort.open("/test-imu/out");
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Opening port "+outputPort.getName()+"...");

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getAxes(&axes), "Cannot get number of controlled axes");
    std::string axisName;
    std::vector<std::string> axis;
    
    for (int i = 0; i < axes; i++)
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iaxes->getAxisName(i, axisName), "Cannot get the name of controlled axes");
        axis.push_back(axisName);
    }
    
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(model.loadReducedModelFromFile(modelPath.c_str(), axis), Asserter::format("Cannot load model %s", modelPath.c_str()));
    kinDynComp.loadRobotModel(model.model());

    iDynTree::Vector3 baseLinkOrientationDeg;
    iDynTree::Vector3 baseLinkOrientationRad;
    baseLinkOrientationDeg.zero();

    for (auto i : baseLinkOrientationDeg)
    {
        baseLinkOrientationRad[i] = iDynTree::deg2rad(baseLinkOrientationDeg[i]);
    }

    baseLinkOrientation = iDynTree::Rotation::RPY(baseLinkOrientationRad[0], baseLinkOrientationRad[1], baseLinkOrientationRad[2]);
    I_T_base = iDynTree::Transform(baseLinkOrientation, iDynTree::Position::Zero());
    iDynTree::Twist baseVelocity = iDynTree::Twist::Zero();

    unsigned int dofs = kinDynComp.model().getNrOfDOFs();
    s.resize(dofs);
    ds.resize(dofs);

    for (int i = 0; i < kinDynComp.model().getNrOfDOFs(); i++) {
        std::cout << kinDynComp.model().getJointName(i) << std::endl;
    }

    positions.resize(axes);
    velocities.resize(axes);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoders(positions.data()), "Cannot get joint positions");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoderSpeeds(velocities.data()), "Cannot get joint positions");

    for(auto i = 0; i < axes; i++)
    {
        s.setVal(i, iDynTree::deg2rad(positions[i]));
        ds.setVal(i, iDynTree::deg2rad(velocities[i]));
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

void Imu::tearDown() {
    outputPort.interrupt();
    outputPort.close();

    if(driver1)
    {
        delete driver1;
        driver1 = 0;
    }

    if(driver2)
    {
        delete driver2;
        driver2 = 0;
    }
}

void Imu::run() {
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Starting reading FT sensors values...");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorMeasureAsRollPitchYaw(0, rpyValues, timestamp), "Unable to obtain rpy measurements.");

    iDynTree::Rotation I_R_FK = kinDynComp.getWorldTransform(frameName).getRotation(); 
    I_R_I_IMU = (I_R_FK * ((iDynTree::Rotation::RPY(iDynTree::deg2rad(rpyValues[0]), iDynTree::deg2rad(rpyValues[1]), iDynTree::deg2rad(rpyValues[2]))).inverse())); 
    
    double minLim;
    double maxLim;

    for (int i = 0; i < 3; i++)
    {
        ilim->getLimits(i, &minLim, &maxLim);

        moveJoint(i, minLim + 5);
        yarp::os::Time::delay(1.0);
        moveJoint(i, maxLim - 5);
        yarp::os::Time::delay(1.0);
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

bool Imu::moveJoint(int ax, double pos)
{
    bool done = false;
    double refPos;

    yarp::os::Time::delay(0.1);

    ipos->positionMove(ax, pos);
    ipos->getTargetPosition(ax, &refPos);

    while(!done)
    {
        iorientation->getOrientationSensorMeasureAsRollPitchYaw(0, rpyValues, timestamp);

        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoders(positions.data()), "Cannot get joint positions");
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoderSpeeds(velocities.data()), "Cannot get joint positions");

        for (auto i = 0; i < axes; i++)
        {
            s.setVal(i, iDynTree::deg2rad(positions[i]));
            ds.setVal(i, iDynTree::deg2rad(velocities[i]));
        }  

        kinDynComp.setRobotState(
        I_T_base,
        s,
        baseVelocity,
        ds,
        gravity);

        iDynTree::Rotation expectedImuSignal = kinDynComp.getWorldTransform(frameName).getRotation();
        iDynTree::Rotation imuSignal = (I_R_I_IMU * iDynTree::Rotation::RPY(iDynTree::deg2rad(rpyValues[0]), iDynTree::deg2rad(rpyValues[1]), iDynTree::deg2rad(rpyValues[2]))); 

        auto error = (expectedImuSignal * imuSignal.inverse()).log();
        double err_mean = (std::accumulate(error.begin(), error.end(), 0)) / error.size();
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(err_mean < 0.1, "Error > 0.1! Aborting ...");

        sendData(expectedImuSignal.asRPY(), imuSignal.asRPY());
        ipos->checkMotionDone(&done);
    }

    ipos->positionMove(ax, 0.0);

    return true;
}