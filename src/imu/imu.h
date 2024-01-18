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