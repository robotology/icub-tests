#include <iostream>
#include <cmath>

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
    
    robotName = property.find("robot").asString(); // robot name on which test
    partName = property.find("part").asString(); // part of the robot on which the sensor is mounted
    modelPath = property.find("model").asString(); // urdf model path
    frameName = property.find("frame").asString(); // frame on which the sensor is attached
    sensorName = property.find("sensor").asString(); // sensor name within urdf

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Running IMU test on "+robotName+" for "+partName);

    yarp::os::Property options1;
    options1.put("device", "multipleanalogsensorsclient");
    options1.put("remote", "/"+robotName+"/"+partName+"/inertials");
    options1.put("local", "/imuTest/"+robotName+"/"+partName);

    yarp::os::Property options2;
    options2.put("device", "remote_controlboard");
    options2.put("remote", "/"+robotName+"/alljoints");
    options2.put("local", "/local");

    driver1 = new yarp::dev::PolyDriver(options1);
    driver2 = new yarp::dev::PolyDriver(options2);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver1->isValid(), "Device driver cannot be opened");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver1->view(iorientation), "Unable to open orientation interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver2->isValid(), "Device driver cannot be opened");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver2->view(ipos), "Unable to open position control interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(driver2->view(ienc), "Unable to open encoder interface");

    yarp::dev::MAS_status status;
    status = iorientation->getOrientationSensorStatus(iorientation->getNrOfOrientationSensors()-1);
    std::cout << "MAS_status: " << status << std::endl; //MAS_status = 4 -> The sensor is read through the network, and the device is waiting to receive the first measurement. 

    outputPort.open("/test-imu/out");
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Opening port "+outputPort.getName()+"...");

    //------------------------------------------------------------------------
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(model.loadModelFromFile(modelPath.c_str()), Asserter::format("Cannot load model %s", modelPath.c_str()));
    
    kinDynComp.loadRobotModel(model.model());
    std::cout << "The loaded model has " << kinDynComp.model().getNrOfDOFs() << " internal degrees of freedom and " << kinDynComp.model().getNrOfLinks() << " links." << std::endl;
    // for(auto i = 0; i < kinDynComp.model().getNrOfDOFs(); i++)
    // {
    //     std::cout << "Joint " << i << " is " << kinDynComp.model().getJointName(i) << std::endl;
    // }

    iDynTree::Vector3 baseLinkOrientationDeg;
    iDynTree::Vector3 baseLinkOrientationRad;
    baseLinkOrientationDeg.zero();

    for (auto i : baseLinkOrientationDeg)
    {
        baseLinkOrientationRad[i] = iDynTree::deg2rad(baseLinkOrientationDeg[i]);
    }

    baseLinkOrientation = iDynTree::Rotation::RPY(baseLinkOrientationRad[0], baseLinkOrientationRad[1], baseLinkOrientationRad[2]);
    iDynTree::Transform I_T_base(baseLinkOrientation, iDynTree::Position::Zero());
    iDynTree::Twist baseVelocity = iDynTree::Twist::Zero();

    unsigned int dofs = kinDynComp.model().getNrOfDOFs();
    s.resize(dofs);
    ds.resize(dofs);

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getAxes(&axes), "Cannot get number of controlled axes");
    for(auto i = 0; i < axes; i++)
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoder(i, &positions), "Cannot get joint positions");
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoderSpeed(i, &velocities), "Cannot get joint positions");

        s.setVal(i, positions);
        ds.setVal(i, velocities);
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
    //-----------------------------------------------------------------------------
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

    std::string imuFrameName;
    std::string imuSensorName;
    double timestamp;

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorName(0, imuSensorName), "Unable to get sensor name");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorFrameName(0, imuFrameName), "Unable to get sensor frame name");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(iorientation->getOrientationSensorMeasureAsRollPitchYaw(0, rpyValues, timestamp), "Unable to obtain rpy measurements.");

    // std::cout << "IMU frame name is: " << imuFrameName << std::endl;
    // std::cout << "IMU sensor name is: " << imuSensorName << std::endl;

    auto count = 0;
    iDynTree::Twist baseVelocity = iDynTree::Twist::Zero();
    iDynTree::Transform I_T_base(baseLinkOrientation, iDynTree::Position::Zero());

    while(true)
    {
        iorientation->getOrientationSensorMeasureAsRollPitchYaw(0, rpyValues, timestamp); //rpy in deg
        sendData(rpyValues);

        count++;
        // yarp::os::Time::delay(0.01);

        //-----------------------------------------------------------
        iDynTree::Rotation I_R_FK = kinDynComp.getWorldTransform(frameName).getRotation(); //compute the orientation of the frame where the IMU is attached 
        iDynTree::Rotation I_R_I_IMU = (I_R_FK * (iDynTree::Rotation::RPY(rpyValues[0], rpyValues[1], rpyValues[2])).inverse()); 

        std::cout << "From FK: " << I_R_I_IMU.asRPY().toString() << std::endl;

        for (auto i = 0; i < axes; i++)
        {
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoder(i, &positions), "Cannot get joint positions");
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(ienc->getEncoderSpeed(i, &velocities), "Cannot get joint positions");
            
            s.setVal(i, positions);
            ds.setVal(i, velocities);
        }  

        // std::cout << "s: " << s.toString() << std::endl;

        kinDynComp.setRobotState(
        I_T_base,
        s,
        baseVelocity,
        ds,
        gravity);

        iDynTree::Rotation expectedImuSignal = kinDynComp.getWorldTransform(frameName).getRotation();
        iDynTree::Rotation imuSignal = (I_R_I_IMU * iDynTree::Rotation::RPY(rpyValues[0], rpyValues[1], rpyValues[2]));

        auto error = (expectedImuSignal * imuSignal.inverse()).log();
        // std::cout << "From FK: " << imuSignal.asRPY().toString() << std::endl;

        //-----------------------------------------------------------
    }

}

bool Imu::sendData(yarp::sig::Vector rpy)
{
    yarp::os::Bottle& out = outputPort.prepare();
    static yarp::os::Stamp stamp;

    stamp.update();
    outputPort.setEnvelope(stamp);
    out.clear();

    out.addFloat64(iDynTree::deg2rad(rpy[0]));
    out.addFloat64(iDynTree::deg2rad(rpy[1]));
    out.addFloat64(iDynTree::deg2rad(rpy[2]));

    out.addFloat64(stamp.getTime());

    outputPort.write();    
    
    return true;
}