// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2018 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _POSITIONACCURACYEXTERNALPID_H_
#define _POSITIONACCURACYEXTERNALPID_H_

#include <string>
#include <yarp/rtf/TestCase.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
//#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>

/**
* \ingroup icub-tests
* This tests checks the response of the system to a position step, sending directly PWM commands to a joint.
* The PWM commands are computed using iCub::ctrl::parallelPID class.
* This test currently does not return any error report. It simply moves a joint, and saves data to a different file for each joint.
* The data acquired can be analyzed with a Matlab script to evaluate the position PID properties.
* Be aware that a step greater than 5 degrees at the maximum speed can be dangerous for both the robot and the human operator!

* example: testRunner -v -t PositionControlAccuracyExternalPid.dll -p "--robot icubSim --part head --joints ""(0 1 2)"" --zeros ""(0 0 0)""  --step 5  --cycles 10 --sampleTime 0.010 --Kp 1.0"
* example: testRunner -v -t PositionControlAccuracyExternalPid.dll -p "--robot icubSim --part head --joints ""(2)"" --zeros ""(0)"" --step 5 --cycles 10 --sampleTime 0.010 --homeTolerance 1.0 --step_duration 8 --Kp 2.2 --Kd 0.01 --Ki 100 --MaxValue 80"

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
* | step_duration      | double | s     | 4     | No  | The duration of the step. After this time, a new test cycle starts. | |
* | Kp                 | double |       | 0     | No  | The Proportional gain | |
* | Ki                 | double |       | 0     | No  | The Integral gain | |
* | Kd                 | double |       | 0     | No  | The Derivative gain | |
* | MaxValue           | double | %     | 100   | No  | max value for PID output (saturator). | |
*
*/

class PositionControlAccuracyExernalPid : public yarp::rtf::TestCase {
public:
    PositionControlAccuracyExernalPid();
    virtual ~PositionControlAccuracyExernalPid();

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
    yarp::dev::IPositionDirect   *idir;
    yarp::dev::IPWMControl       *ipwm;

    iCub::ctrl::parallelPID      *ppid;

    double m_pospid_vup;
    double m_pospid_vdown;


    double  m_cmd_single;
    double* m_encoders;
    std::string  m_requested_filename;
    double m_home_tolerance;
    double m_step_duration;
};

#endif
