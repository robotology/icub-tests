#ifndef _IMU_H_
#define _IMU_H_

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
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
        std::string partName;
        std::string portName;
        std::string modelPath;
        std::string frameName;
        std::string sensorName;

        yarp::dev::PolyDriver *driver1;
        yarp::dev::PolyDriver *driver2;
        yarp::dev::IOrientationSensors* iorientation;
        yarp::dev::IPositionControl* ipos;
        yarp::dev::IEncoders* ienc;

        yarp::os::BufferedPort <yarp::os::Bottle> outputPort;
        yarp::sig::Vector rpyValues;
        double positions;
        double velocities;
        int axes;

        iDynTree::ModelLoader model;
        iDynTree::KinDynComputations kinDynComp;

        iDynTree::VectorDynSize s;
        iDynTree::VectorDynSize ds;
        iDynTree::Vector3 gravity;
        iDynTree::Rotation baseLinkOrientation;

        bool sendData(iDynTree::Vector3 expectedValues, iDynTree::Vector3 imuSignal);

    };

#endif //_IMU_H_
