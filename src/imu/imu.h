#ifndef IMU_H
#define IMU_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/robottestingframework/TestCase.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Model.h>

/**
* \ingroup icub-tests
*
* The purpose of this test is to evaluate the accuracy of the IMU Euler angles measurements.
* It takes as input the urdf of the robot and make a comparison between the expected values retrieved from the forward kinematics and the ones read from the IMU itself.
* The test involves the movements of the joints belonging to the part on which the sensors are mounted.
*
* You can find the parameters involved in the test in suites/contexts/icub/test_imu.ini file. 
* To run the test, you have to install <a href="https://pixi.sh/latest/#installation">pixi</a> and then, after cloning this repo, you can run:
*
* pixi run imu_test
*
* This will compile the dependencies, launch the test and, at the end, a .mat file is generated containing the relevant measurements of the test.
*
*  Accepts the following parameters:
* | Parameter name     | Type               | Units | Default Value | Required | Description | Notes |
* |:------------------:|:------------------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | robot              | string             | -     | -             | Yes      | The name of the robot. | e.g. icub |
* | model              | string             | -     | -             | Yes      | The name of the robot model. | e.g. model.urdf |
* | port               | string             | -     | -             | Yes      | The name of the port streaming IMU data. | e.g. /icub/head/inertials |
* | remoteControlBoards| vector of string   | -     | -             | Yes      | The list of the controlboards to open. | e.g. ("torso", "head") |
* | axesNames          | vector of string   | -     | -             | Yes      | The list of the controlled joints. | e.g. ("torso_pitch", "torso_roll", "torso_yaw", "neck_pitch", "neck_roll", "neck_yaw") |
* | sensorsList        | vector of string   | -     | -             | Yes      | The list of the sensors to be tested | e.g. ("head_imu_0", "l_arm_ft") or ("all)|
* | maxError           | double             | -     | -             | Yes      | The tolerance on the error. | |
*/

class Imu : public yarp::robottestingframework::TestCase {
    public:
        Imu();
        virtual ~Imu();
        virtual bool setup(yarp::os::Property& property);
        virtual void tearDown();
        virtual void run();

    private:
        std::string robotName;
        std::string portName;
        std::string partName;
        std::string modelName;
        std::string frameName;
        std::string sensorName;
        double errorMax;
        yarp::os::Bottle sensorsList;

        yarp::dev::PolyDriver MASclientDriver;
        yarp::dev::PolyDriver controlBoardDriver;
        yarp::dev::PolyDriver MASremapperDriver;
        yarp::dev::IOrientationSensors* iorientation;
        yarp::dev::IPositionControl* ipos;
        yarp::dev::IEncoders* ienc;
        yarp::dev::IAxisInfo* iaxes;
        yarp::dev::IMultipleWrapper* imultiwrap;

        yarp::os::BufferedPort <yarp::os::Bottle> outputPort;
        std::vector<yarp::sig::Vector> rpyValues;
        yarp::sig::Vector positions;
        yarp::sig::Vector velocities;

        int axes;
        double timestamp;
        std::vector<std::string> axesVec;

        iDynTree::ModelLoader model;
        iDynTree::KinDynComputations kinDynComp;
        iDynTree::VectorDynSize s;
        iDynTree::VectorDynSize ds;
        iDynTree::Vector3 gravity;
        iDynTree::Rotation baseLinkOrientation;
        iDynTree::Twist baseVelocity;
        iDynTree::Transform I_T_base;
        iDynTree::Rotation I_R_I_IMU;

        robometry::BufferManager bufferManager;

        bool sendData(iDynTree::Vector3 expectedValues, iDynTree::Vector3 imuSignal);
        bool startMove();
        bool setupRobometry();
    };

#endif //IMU_H