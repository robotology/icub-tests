#ifndef _IMU_H_
#define _IMU_H_

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/robottestingframework/TestCase.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Model.h>

struct SensorInfo {
    std::string sensorName{" "};
    std::string frameName{" "};
    iDynTree::Transform transform{iDynTree::Transform::Identity()};
};

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

    yarp::dev::PolyDriver *driver;
    yarp::dev::IOrientationSensors* iorientation;

    yarp::os::BufferedPort <yarp::os::Bottle> outputPort;
    yarp::sig::Vector rpyValues;

    iDynTree::ModelLoader model;
    iDynTree::KinDynComputations kinDynComp;

};
#endif //_IMU_H_
