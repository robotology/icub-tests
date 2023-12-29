#include <iostream>
#include <cmath>

#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>

#include <yarp/os/Property.h>

#include <eigen3/Eigen/Dense>
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
    
    robotName = property.find("robot").asString(); // robot name on which test
    partName = property.find("part").asString(); // part of the robot on which the sensor is mounted
    modelPath = property.find("model").asString(); // urdf model path
    frameName = property.find("frame").asString(); // frame on which the sensor is attached
    sensorName = property.find("sensor").asString(); // sensor name within urdf

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Running IMU test on "+robotName+" for "+partName);

    yarp::os::Property options;
    options.put("device", "multipleanalogsensorsclient");
    options.put("remote", "/"+robotName+"/"+partName+"/inertials");
    options.put("local", "/imuTest/"+robotName+"/"+partName);

    driver = new yarp::dev::PolyDriver(options);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver->isValid(), "Device driver cannot be opened");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver->view(iorientation), "Unable to open orientation interface");

    size_t m_imuSensorIndex = 0;
    std::string name;

    auto nbSensors = iorientation->getNrOfOrientationSensors();	
    std::cout << nbSensors << std::endl;
    yarp::dev::MAS_status status;
    status = iorientation->getOrientationSensorStatus(m_imuSensorIndex);
    std::cout << "MAS_status: " << status << std::endl; //MAS_status = 4 -> The sensor is read through the network, and the device is waiting to receive the first measurement. 

    outputPort.open("/test-imu/out");


    // while(true);
    // {
    //     out.addFloat64(rpyValues[0]);
    //     outputPort.write(&out);
    //     std::cout << out.toString().c_str() << std::endl;

    //     yarp::os::Time::delay(0.1);

    // }

    //------------------------------------------------------------------------
    bool ok = model.loadModelFromFile(modelPath.c_str());
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ok, Asserter::format("Cannot load model %s", modelPath.c_str()));
    
    kinDynComp.loadRobotModel(model.model());
    std::cout << "The loaded model has " << kinDynComp.model().getNrOfDOFs() << " internal degrees of freedom and " << kinDynComp.model().getNrOfLinks() << " links." << std::endl;
    std::array<double, 3> baseLinkOrientationDeg {0.0, 0.0, 0.0};
    std::array<double, 3> baseLinkOrientationRad;
    for (auto i : baseLinkOrientationDeg)
    {
        baseLinkOrientationRad[i] = baseLinkOrientationDeg[i] * M_PI/180;
    }

    auto baseLinkOrientation = iDynTree::Rotation::RPY(baseLinkOrientationRad[0], baseLinkOrientationRad[1], baseLinkOrientationRad[2]);
    
    iDynTree::Transform I_T_base(baseLinkOrientation, iDynTree::Position::Zero());
    iDynTree::Twist baseVelocity = iDynTree::Twist::Zero();
    unsigned int dofs = kinDynComp.model().getNrOfDOFs();
    iDynTree::VectorDynSize s(dofs), ds(dofs);

    std::cout << ds.toString() << std::endl;

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;

    kinDynComp.setRobotState(
        I_T_base,
        s,
        baseVelocity,
        ds,
        gravity
    );
    
    //-----------------------------------------------------------------------------
    return true;
}

void Imu::tearDown() {
    outputPort.close();

    if(driver)
    {
        delete driver;
        driver = 0;
    }
}

void Imu::run() {
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Starting reading FT sensors values...");

    std::string imuFrameName;
    std::string imuSensorName;
    double timestamp;

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorName(0, imuSensorName), "Unable to get sensor name");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorFrameName(0, imuFrameName), "Unable to get sensor frame name");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorMeasureAsRollPitchYaw(0, rpyValues, timestamp), "Unable to obtain rpy measurements.");

    std::cout << "IMU frame name is: " << imuFrameName << std::endl;
    std::cout << "IMU sensor name is: " << imuSensorName << std::endl;

    yarp::os::Bottle& out = outputPort.prepare();
    out.clear();
    auto count = 0;
    while(count < 100)
    {
        iorientation->getOrientationSensorMeasureAsRollPitchYaw(0, rpyValues, timestamp);
        out.addFloat64(rpyValues[0]);
        out.addFloat64(rpyValues[1]);
        out.addFloat64(rpyValues[2]);

        // std::cout << out.toString().c_str() << std::endl;
        outputPort.write();
        count++;
        yarp::os::Time::delay(0.01);

        //-----------------------------------------------------------
        iDynTree::Rotation I_R_FK = kinDynComp.getWorldTransform(frameName).getRotation();
        iDynTree::Rotation I_R_I_IMU = (I_R_FK * (iDynTree::Rotation::RPY(rpyValues[0], rpyValues[1], rpyValues[2])).inverse()); 
        
        iDynTree::Rotation expectedImuSignal = kinDynComp.getWorldTransform(frameName).getRotation();
        iDynTree::Rotation imuSignal = (I_R_I_IMU * iDynTree::Rotation::RPY(rpyValues[0], rpyValues[1], rpyValues[2]));

        auto error = (expectedImuSignal * imuSignal.inverse()).log();

        //-----------------------------------------------------------

        out.clear();
    }

}
