/*
 * iCub Robot Unit Tests (Robot Testing Framework)
 *
 * Copyright (C) 2015-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */


#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#include "PositionControlAccuracy.h"

using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::dev;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(PositionControlAccuracy)

PositionControlAccuracy::PositionControlAccuracy() : yarp::robottestingframework::TestCase("PositionControlAccuracy") {
    m_jointsList = 0;
    m_encoders = 0;
    m_zeros = 0;
    dd=0;
    ipos=0;
    icmd=0;
    iimd=0;
    ienc=0;
    idir=0;
    m_home_tolerance=0.5;
    m_step_duration=4;
}

PositionControlAccuracy::~PositionControlAccuracy() { }

bool PositionControlAccuracy::setup(yarp::os::Property& property) {

    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("robot"), "The robot name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("part"), "The part name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("joints"), "The joints list must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("zeros"),    "The zero position list must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("cycles"), "The number of cycles of the control signal must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("step"), "The amplitude of the step reference signal expressed in degrees!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("sampleTime"), "The sampleTime of the control signal must be given as the test parameter!");
    if(property.check("filename"))
      {m_requested_filename = property.find("filename").asString();}
    if(property.check("home_tolerance"))
      {m_home_tolerance = property.find("home_tolerance").asFloat64();}
    if(property.check("step_duration"))
      {m_step_duration = property.find("step_duration").asFloat64();}

    m_robotName = property.find("robot").asString();
    m_partName = property.find("part").asString();

    Bottle* jointsBottle = property.find("joints").asList();
    Bottle* zerosBottle = property.find("zeros").asList();

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(jointsBottle!=0,"unable to parse joints parameter");
    m_n_cmd_joints = jointsBottle->size();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(m_n_cmd_joints>0, "invalid number of joints, it must be >0");

    m_step = property.find("step").asFloat64();

    m_cycles = property.find("cycles").asInt32();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(m_cycles>0, "invalid cycles");

    m_sampleTime = property.find("sampleTime").asFloat64();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(m_sampleTime>0, "invalid sampleTime");

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/" + m_robotName + "/" + m_partName);
    options.put("local", "/positionControlAccuracyTest/" + m_robotName + "/" + m_partName);

    dd = new PolyDriver(options);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->isValid(),"Unable to open device driver");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(idir),"Unable to open position direct interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ienc),"Unable to open encoders interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ipos),"Unable to open position interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(icmd),"Unable to open control mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iimd),"Unable to open interaction mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ipid),"Unable to open pid interface");

    if (!ienc->getAxes(&m_n_part_joints))
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    m_zeros = new double[m_n_part_joints];
    m_encoders = new double[m_n_part_joints];
    m_jointsList = new int[m_n_cmd_joints];
    for (int i = 0; i <m_n_cmd_joints; i++) m_jointsList[i] = jointsBottle->get(i).asInt32();
    for (int i = 0; i <m_n_cmd_joints; i++) m_zeros[i] = zerosBottle->get(i).asFloat64();

    double p_Kp=std::nanf("");
    double p_Ki=std::nanf("");
    double p_Kd=std::nanf("");
    double p_Max=std::nanf("");

    int cj=m_jointsList[0];
    ipid->getPid(VOCAB_PIDTYPE_POSITION,cj,&m_orig_pid);
    m_new_pid=m_orig_pid;

    if(property.check("Kp"))
      {p_Kp = property.find("Kp").asFloat64();}
    if(property.check("Ki"))
      {p_Ki = property.find("Ki").asFloat64();}
    if(property.check("Kd"))
      {p_Kd = property.find("Kd").asFloat64();}
    //if(property.check("MaxValue"))
    //  {p_Max = property.find("MaxValue").asFloat64();}
    if (std::isnan(p_Kp)==false) {m_new_pid.kp=p_Kp;}
    if (std::isnan(p_Kd)==false) {m_new_pid.kd=p_Kd;}
    if (std::isnan(p_Ki)==false) {m_new_pid.ki=p_Ki;}

    /*if (std::isnan(p_Kp)==false ||
        std::isnan(p_Ki)==false ||
        std::isnan(p_Kd)==false)
    {
        ipid->setPid(cj,m_new_pid);
    }*/

    return true;
}

void PositionControlAccuracy::tearDown()
{
    if (m_jointsList) { delete [] m_jointsList; m_jointsList = 0; }
    if (m_zeros) { delete [] m_zeros; m_zeros = 0; }
    if (m_encoders) { delete [] m_encoders; m_encoders = 0; }
    if (dd) {delete dd; dd =0;}
}

void PositionControlAccuracy::setMode(int desired_mode)
{
    for (int i = 0; i<m_n_cmd_joints; i++)
    {
        icmd->setControlMode(m_jointsList[i], desired_mode);
        iimd->setInteractionMode(m_jointsList[i], VOCAB_IM_STIFF);
        yarp::os::Time::delay(0.010);
    }

    int cmode;
    yarp::dev::InteractionModeEnum imode;
    int timeout = 0;

    while (1)
    {
        int ok=0;
        for (int i = 0; i<m_n_cmd_joints; i++)
        {
            icmd->getControlMode(m_jointsList[i], &cmode);
            iimd->getInteractionMode(m_jointsList[i], &imode);
            if (cmode==desired_mode && imode==VOCAB_IM_STIFF) ok++;
        }
        if (ok == m_n_cmd_joints) break;
        if (timeout>100)
        {
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("Unable to set control mode/interaction mode");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

bool PositionControlAccuracy::goHome()
{
    for (int i = 0; i<m_n_cmd_joints; i++)
    {
        ipos->setRefSpeed(m_jointsList[i], 20.0);
        ipos->positionMove(m_jointsList[i], m_zeros[i]);
    }

    int timeout = 0;
    while (1)
    {
        int in_position=0;
        for (int i = 0; i<m_n_cmd_joints; i++)
        {
            ienc->getEncoder(m_jointsList[i], &m_encoders[m_jointsList[i]]);
            if (fabs(m_encoders[m_jointsList[i]] - m_zeros[i])<m_home_tolerance) in_position++;
        }
        if (in_position == m_n_cmd_joints) break;
        if (timeout>100)
        {
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("Timeout while reaching zero position");
            return false;
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
    //sleep some additional time to complete movement from m_home_tolerance to 0
    yarp::os::Time::delay(0.5);
    return true;
}

void PositionControlAccuracy::run()
{
    for (int i = 0; i < m_n_cmd_joints; i++)
    {
        for (int cycle = 0; cycle < m_cycles; cycle++)
        {
			ipid->setPid(VOCAB_PIDTYPE_POSITION,m_jointsList[i],m_orig_pid);
            setMode(VOCAB_CM_POSITION);
            if (goHome() == false)
            {
                ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Test stopped");
            };

            ipid->setPid(VOCAB_PIDTYPE_POSITION,m_jointsList[i],m_new_pid);
            setMode(VOCAB_CM_POSITION_DIRECT);
            double start_time = yarp::os::Time::now();

            char cbuff[64];
            sprintf(cbuff, "Testing Joint: %d cycle: %d", i, cycle);

            std::string buff(cbuff);
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(buff);

            double time_zero = 0;
            yarp::os::Bottle      dataToPlotRaw;
            yarp::os::Bottle      dataToPlotSync;

            while (1)
            {
                double curr_time = yarp::os::Time::now();
                double elapsed = curr_time - start_time;

                if (elapsed <= 1.0)
                {
                    m_cmd_single = m_zeros[i]; //0.0;
                }
                else if (elapsed > 1.0 && elapsed <= m_step_duration)
                {
                    m_cmd_single = m_zeros[i] + m_step;
                    if (time_zero == 0) time_zero = elapsed;
                }
                else
                {
                    break;
                }

                ienc->getEncoders(m_encoders);
                idir->setPosition(m_jointsList[i], m_cmd_single);

                Bottle& b1 = dataToPlotRaw.addList();
                b1.addInt32(cycle);
                b1.addFloat64(elapsed);
                b1.addFloat64(m_encoders[m_jointsList[i]]);
                b1.addFloat64(m_cmd_single);
                yarp::os::Time::delay(m_sampleTime);
            }

            //reorder data
            for (int t = 0; t < dataToPlotRaw.size(); t++)
            {
                int    cycle = dataToPlotRaw.get(t).asList()->get(0).asInt32();
                double time = dataToPlotRaw.get(t).asList()->get(1).asFloat64();
                double val = dataToPlotRaw.get(t).asList()->get(2).asFloat64();
                double cmd = dataToPlotRaw.get(t).asList()->get(3).asFloat64();
                Bottle& b1 = dataToPlotSync.addList();
                b1.addInt32(cycle);
                b1.addFloat64(time - time_zero);
                b1.addFloat64(val);
                b1.addFloat64(cmd);
            }

            m_dataToSave.append(dataToPlotSync);
        } //cycle loop

        //save data
        std::string filename;
        if (m_requested_filename=="")
        {
            char cfilename[128];
            sprintf(cfilename, "positionControlAccuracy_plot_%s%d.txt", m_partName.c_str(), i);
            filename = cfilename;
            //filename += m_partName;
            //filename += std::to_string(i);
            //filename += ".txt";
        }
        else
        {
            filename=m_requested_filename;
        }
        yInfo() << "Saving file to: "<< filename;
        saveToFile(filename, m_dataToSave);
        ipid->setPid(VOCAB_PIDTYPE_POSITION,m_jointsList[i],m_orig_pid);
    } //joint loop

    //data acquisition ends here
    setMode(VOCAB_CM_POSITION);
    goHome();
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Data acquisition complete");

    //plot data
    /*for (int i = 0; i < m_n_cmd_joints; i++)
    {
        std::string filename = "positionControlAccuracy_plot_";
        filename += m_partName;
        filename += std::to_string(i);
        filename += ".txt";
        char plotstring[1000];
        sprintf(plotstring, "gnuplot -e \" unset key; plot for [col=1:%d] '%s' using col with lines \" -persist", m_n_cmd_joints, filename.c_str());
        system(plotstring);
    }*/
}

void PositionControlAccuracy::saveToFile(std::string filename, yarp::os::Bottle &b)
{
    std::fstream fs;
    fs.open(filename.c_str(), std::fstream::out);

    for (int i = 0; i<b.size(); i++)
    {
        std::string s = b.get(i).toString();
        std::replace(s.begin(), s.end(), '(', ' ');
        std::replace(s.begin(), s.end(), ')', ' ');
        fs << s << std::endl;
    }

    fs.close();
}
