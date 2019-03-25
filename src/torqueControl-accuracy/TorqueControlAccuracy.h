// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _TORQUEACCURACY_H_
#define _TORQUEACCURACY_H_

#include <string>
#include <yarp/rtf/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

/**
* \ingroup icub-tests
* This tests checks the a torque PID response, sending a step reference signal with a setRefTorque command.
* This test currently does not return any error report. It simply moves a joint, and saves data to a different file for each joint.
* The data acquired can be analized with a matalab script to evaluate the torque PID properties.
* Be aware that a step greater than 1 Nm may be dangerous for both the robot and the human operator!

* example: testRunner -v -t TorqueControlAccuracy.dll -p "--robot icubSim --part head --joints ""(0 1 2)"" --zeros ""(0 0 0)""  --step 5  --cycles 10 --sampleTime 0.010"
* example: testRunner -v -t TorqueControlAccuracy.dll -p "--robot icubSim --part head --joints ""(2)"" --zeros ""(0)"" --step 5 --cycles 10 --sampleTime 0.010"

*  Accepts the following parameters:
* | Parameter name     | Type   | Units | Default Value | Required | Description | Notes |
* |:------------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | robot              | string | -     | -     | Yes | The name of the robot.     | e.g. icub |
* | part               | string | -     | -     | Yes | The name of the robot part. | e.g. left_arm |
* | joints             | vector of ints | - | - | Yes | List of joints to be tested | |
* | zeros              | double | deg   | -     | Yes | The home position for each joint | |
* | cycles             | int    | -     | -     | Yes | Each joint will be tested multiple times |   |
* | step               | double | Nm    | -     | Yes | The amplitude of the step reference signal | Recommended max: 1 Nm! |
* | sampleTime         | double | s     | -     | Yes | The sample time of the control thread | |
*
*/

class TorqueControlAccuracy : public yarp::rtf::TestCase {
public:
    TorqueControlAccuracy();
    virtual ~TorqueControlAccuracy();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

    bool goHome();
    void executeCmd();
    void setMode(int desired_mode);
    void saveToFile(std::string filename, yarp::os::Bottle &b);

private:
    std::string m_robotName;
    std::string m_partName;
    int*        m_jointsList;
    int         m_cycles;
    double      m_sampleTime;
    double*     m_zeros;
    double      m_step;
    int         m_n_part_joints;
    int         m_n_cmd_joints;
    yarp::os::Bottle      m_dataToSave;

    yarp::dev::PolyDriver        *dd;
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IControlMode2     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::ITorqueControl    *itrq;

    double  m_cmd_single;
    double* m_encoders;
    double* m_torques;
};

#endif
