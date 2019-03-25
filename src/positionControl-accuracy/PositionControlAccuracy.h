// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _POSITIONACCURACY_H_
#define _POSITIONACCURACY_H_

#include <string>
#include <yarp/rtf/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

/**
* \ingroup icub-tests
* This tests checks the a position PID response, sending a step reference signal with a positionDirect command.
* This test currently does not return any error report. It simply moves a joint, and saves data to a different file for each joint.
* The data acquired can be analyzed with a Matlab script to evaluate the position PID properties.
* Be aware that a step greater than 5 degrees at the maximum speed can be dangerous for both the robot and the human operator!

* example: testRunner -v -t PositionControlAccuracy.dll -p "--robot icubSim --part head --joints ""(0 1 2)"" --zeros ""(0 0 0)""  --step 5  --cycles 10 --sampleTime 0.010"
* example: testRunner -v -t PositionControlAccuracy.dll -p "--robot icubSim --part head --joints ""(2)"" --zeros ""(0)"" --step 5 --cycles 10 --sampleTime 0.010"

*  Accepts the following parameters:
* | Parameter name     | Type   | Units | Default Value | Required | Description | Notes |
* |:------------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | robot              | string | -     | -     | Yes | The name of the robot.     | e.g. icub |
* | part               | string | -     | -     | Yes | The name of the robot part. | e.g. left_arm |
* | joints             | vector of ints | - | - | Yes | List of joints to be tested | |
* | zeros              | double | deg   | -     | Yes | The home position for each joint | |
* | cycles             | int    | -     | -     | Yes | Each joint will be tested multiple times |   |
* | step               | double | deg   | -     | Yes | The amplitude of the step reference signal | Recommended max: 5 deg! |
* | sampleTime         | double | s     | -     | Yes | The sample time of the control thread | |
* | home_tolerance     | double | deg   | 0.5   | No  | The max acceptable position error during the homing phase. | |
* | filename           | string |       |       | No  | The output filename. If not specified, the name will be generated using 'part' parameter and joint number | |
* | step_duration      | double | s     |       | No  | The duration of the step. After this time, a new test cycle starts. | |
*
*/

class PositionControlAccuracy : public yarp::rtf::TestCase {
public:
    PositionControlAccuracy();
    virtual ~PositionControlAccuracy();

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
    yarp::dev::IControlMode     *icmd;
    yarp::dev::IInteractionMode  *iimd;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IPositionDirect   *idir;
    yarp::dev::IPidControl       *ipid;

    double  m_cmd_single;
    double* m_encoders;
    std::string m_requested_filename;
    double m_home_tolerance;
    double m_step_duration;
    yarp::dev::Pid m_orig_pid;
    yarp::dev::Pid m_new_pid;    
};

#endif
