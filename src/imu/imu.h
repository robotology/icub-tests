#ifndef IMU_H
#define IMU_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/robottestingframework/TestCase.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Model.h>

/**
* \ingroup icub-tests
*
* The purpose of this test is to evaluate the accuracy of the IMU Euler angles measurements.
* It takes as input the urdf of the robot and make a comparison between the expected values retrieved from the forward kinematics and the ones read from the IMU itself.
* The test involves the movements of the joints belonging to the part on which the sensor is mounted. The movements are executed sequentially, traversing from the home position to the lower limit, upper limit and back to the home position for each joint.
*
* Example: robottestingframework-testrunner --test plugins/imu.so --param "--robot icub --model model.urdf --port /icub/head/inertials --part head --controlboards ("torso", "head") --sensor head_imu_0 --frame head_imu_0 --mean_error 0.1"
*
* Moreover, you can manually modify the suites/contexts/icub/test_imu.ini file depending on the parameters of the test. In this case, after compiling, you can run:
*
* robottestingframework-testrunner --suite ../suites/imu.xml
*
* This will launch the test and open a yarpscope with the plots of the IMU traces.
*
*  Accepts the following parameters:
* | Parameter name     | Type               | Units | Default Value | Required | Description | Notes |
* |:------------------:|:------------------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | robot              | string             | -     | -             | Yes      | The name of the robot. | e.g. icub |
* | model              | string             | -     | -             | Yes      | The name of the robot model. | e.g. model.urdf |
* | port               | string             | -     | -             | Yes      | The name of the port streaming IMU data. | e.g. /icub/head/inertials |
* | part               | string             | -     | -             | Yes      | The name of the robot part on which the sensor is mounted. | e.g. head |
* | controlboards      | vector of string   | -     | -             | Yes      | The list of the controlboards to open. | e.g. ("torso", "head") |
* | sensor             | string             | -     | -             | Yes      | The name of the sensor to be tested. | e.g. head_imu_0 |
* | frame              | string             | -     | -             | Yes      | The name of the frame on which the sensor is attached. | e.g. head_imu_0|
* | mean_error         | double             | -     | -             | Yes      | The tolerance on the mean of the error. | |
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
        double errorMean;

        yarp::dev::PolyDriver MASclientDriver;
        yarp::dev::PolyDriver controlBoardDriver;
        yarp::dev::IOrientationSensors* iorientation;
        yarp::dev::IPositionControl* ipos;
        yarp::dev::IEncoders* ienc;
        yarp::dev::IAxisInfo* iaxes;
        yarp::dev::IControlLimits* ilim;

        yarp::os::BufferedPort <yarp::os::Bottle> outputPort;
        yarp::sig::Vector rpyValues;
        yarp::sig::Vector positions;
        yarp::sig::Vector velocities;

        int axes;
        double timestamp;
        std::vector<std::string> axis;

        iDynTree::ModelLoader model;
        iDynTree::KinDynComputations kinDynComp;
        iDynTree::VectorDynSize s;
        iDynTree::VectorDynSize ds;
        iDynTree::Vector3 gravity;
        iDynTree::Rotation baseLinkOrientation;
        iDynTree::Twist baseVelocity;
        iDynTree::Transform I_T_base;
        iDynTree::Rotation I_R_I_IMU;

        bool sendData(iDynTree::Vector3 expectedValues, iDynTree::Vector3 imuSignal);
        bool moveJoint(int ax, double pos);
    };

#endif //IMU_H