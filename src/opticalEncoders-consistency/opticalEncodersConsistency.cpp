// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <math.h>
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include "opticalEncodersConsistency.h"
#include <yarp/manager/localbroker.h>

//example     -v -t OpticalEncodersConsistency.dll -p "--robot icub --part left_arm --joints ""(0 1 2)"" --home ""(-30 30 10)"" --speed ""(20 20 20)"" --max ""(-20 40 20)"" --min ""(-40 20 0)"" --cycles 10 --tolerance 1.0 "
//example2    -v -t OpticalEncodersConsistency.dll -p "--robot icub --part head     --joints ""(2)""     --home ""(0)""         --speed ""(20      )"" --max ""(10      )""  --min ""(-10)""      --cycles 10 --tolerance 1.0 "
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

// prepare the plugin
PREPARE_PLUGIN(OpticalEncodersConsistency)

OpticalEncodersConsistency::OpticalEncodersConsistency() : YarpTestCase("OpticalEncodersConsistency") {
    jointsList=0;
    dd=0;
    ipos=0;
    icmd=0;
    iimd=0;
    ienc=0;
    imot=0;
    enc_jnt=0;
    enc_jnt2mot=0;
    enc_mot=0;
    vel_jnt=0;
    vel_jnt2mot=0;
    vel_mot=0;
    acc_jnt=0;
    acc_jnt2mot=0;
    acc_mot=0;
    cycles =10;

    matrix_arms.resize(16,16);
    matrix_arms.eye();
    double r=40;
    double R=65;
    matrix_arms (0,0) = 1;
    matrix_arms (1,0) = -R/r;
    matrix_arms (1,1) = R/r;
    matrix_arms (2,0) = -R/r;
    matrix_arms (2,1) = R/r;
    matrix_arms (2,2) = R/r;

    matrix_torso.resize(3,3);
    matrix_torso.eye();
    r=0.022*1000;
    R=0.04*1000;
    matrix_torso (0,0) = R/r;
    matrix_torso (1,0) = 0;
    matrix_torso (2,0) = 0;
    matrix_torso (0,1) = -1;
    matrix_torso (1,1) = 1;
    matrix_torso (2,1) = 1;
    matrix_torso (0,2) = 0;
    matrix_torso (1,2) = -1;
    matrix_torso (2,2) = 1;

    matrix_legs.resize(6,6);
    matrix_legs.eye();
    matrix_legs (0,0) = 60/40;

    matrix_head.resize(6,6);
    matrix_head.eye();
}

OpticalEncodersConsistency::~OpticalEncodersConsistency() { }

bool OpticalEncodersConsistency::setup(yarp::os::Property& property) {

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"), "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"), "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("home"),      "The home position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("max"),       "The max position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("min"),       "The min position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("speed"),     "The positionMove reference speed must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("tolerance"), "The tolerance of the control signal must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* homeBottle = property.find("home").asList();
    RTF_ASSERT_ERROR_IF(homeBottle!=0,"unable to parse home parameter");

    Bottle* maxBottle = property.find("max").asList();
    RTF_ASSERT_ERROR_IF(maxBottle!=0,"unable to parse max parameter");

    Bottle* minBottle = property.find("min").asList();
    RTF_ASSERT_ERROR_IF(minBottle!=0,"unable to parse min parameter");

    Bottle* speedBottle = property.find("speed").asList();
    RTF_ASSERT_ERROR_IF(speedBottle!=0,"unable to parse speed parameter");

    tolerance = property.find("tolerance").asDouble();
    RTF_ASSERT_ERROR_IF(tolerance>=0,"invalid tolerance");

    //optional parameters
    if (property.check("cycles"))
    {cycles = property.find("cycles").asInt();}

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/positionDirectTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(imot),"Unable to open motor encoders interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    enc_jnt.resize(n_part_joints);
    enc_mot.resize(n_part_joints);
    vel_jnt.resize(n_part_joints);
    vel_mot.resize(n_part_joints);
    acc_jnt.resize(n_part_joints);
    acc_mot.resize(n_part_joints);
    zero_vector.resize(n_part_joints);
    zero_vector.zero();

    max.resize(n_cmd_joints);   for (int i=0; i< n_cmd_joints; i++) max[i]=maxBottle->get(i).asDouble();
    min.resize(n_cmd_joints);   for (int i=0; i< n_cmd_joints; i++) min[i]=minBottle->get(i).asDouble();
    home.resize(n_cmd_joints);  for (int i=0; i< n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    speed.resize(n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) speed[i]=speedBottle->get(i).asDouble();

    return true;
}

void OpticalEncodersConsistency::tearDown()
{
    if (dd) {delete dd; dd =0;}
}

void OpticalEncodersConsistency::setMode(int desired_mode)
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        icmd->setControlMode((int)jointsList[i],desired_mode);
        iimd->setInteractionMode((int)jointsList[i],VOCAB_IM_STIFF);
        yarp::os::Time::delay(0.010);
    }

    int cmode;
    yarp::dev::InteractionModeEnum imode;
    int timeout = 0;

    while (1)
    {
        int ok=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            icmd->getControlMode ((int)jointsList[i],&cmode);
            iimd->getInteractionMode((int)jointsList[i],&imode);
            if (cmode==desired_mode && imode==VOCAB_IM_STIFF) ok++;
        }
        if (ok==jointsList.size()) break;
        if (timeout>100)
        {
            RTF_ASSERT_ERROR("Unable to set control mode/interaction mode");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

void OpticalEncodersConsistency::goHome()
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipos->setRefSpeed((int)jointsList[i],speed[i]);
        ipos->positionMove((int)jointsList[i],home[i]);
    }

    int timeout = 0;
    while (1)
    {
        int in_position=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            double tmp=0;
            ienc->getEncoder((int)jointsList[i],&tmp);
            if (fabs(tmp-home[i])<0.5) in_position++;
        }
        if (in_position==jointsList.size()) break;
        if (timeout>100)
        {
            RTF_ASSERT_ERROR("Timeout while reaching home position");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

void OpticalEncodersConsistency::saveToFile(std::string filename, yarp::os::Bottle &b)
{
    std::fstream fs;
    fs.open (filename.c_str(), std::fstream::out);
    
    for (int i=0; i<b.size(); i++)
    {
        std::string s = b.get(i).toString();
        std::replace(s.begin(), s.end(), '(', ' ');
        std::replace(s.begin(), s.end(), ')', ' ');
        fs << s << endl;
    }

    fs.close();
}

void OpticalEncodersConsistency::run()
{
    setMode(VOCAB_CM_POSITION);
    goHome();

    bool go_to_max=false;
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipos->positionMove(i,min[i]);
    }

    int  cycle=0;
    double start_time = yarp::os::Time::now();

    if      (partName=="left_arm")  {matrix = matrix_arms;}
    else if (partName=="right_arm") {matrix = matrix_arms;}
    else if (partName=="torso")     {matrix = matrix_torso;}
    else if (partName=="left_leg")  {matrix = matrix_legs;}
    else if (partName=="right_leg") {matrix = matrix_legs;}
    else if (partName=="head")      {matrix = matrix_head;}
    else
    {
        RTF_ASSERT_ERROR("Unknown part name, missing matrix for this part");
    }
    trasp_matrix = matrix.transposed();
    inv_matrix = yarp::math::luinv(matrix);
    inv_trasp_matrix = inv_matrix.transposed();
    Bottle dataToPlot_test1;
    Bottle dataToPlot_test2;
    Bottle dataToPlot_test3;
    Bottle dataToPlot_test4;

    bool test_data_is_valid = false;

    while(1)
    {
        double curr_time = yarp::os::Time::now();
        double elapsed = curr_time-start_time;

        ienc->getEncoders                  (enc_jnt.data());
        imot->getMotorEncoders             (enc_mot.data());
        ienc->getEncoderSpeeds             (vel_jnt.data());
        imot->getMotorEncoderSpeeds        (vel_mot.data());
        ienc->getEncoderAccelerations      (acc_jnt.data());
        imot->getMotorEncoderAccelerations (acc_mot.data());

        if (enc_jnt==zero_vector) {RTF_TEST_REPORT("Invalid getEncoders data");test_data_is_valid=true;}
        if (enc_mot==zero_vector) {RTF_TEST_REPORT("Invalid getMotorEncoders data");test_data_is_valid=true;}
        if (vel_jnt==zero_vector) {RTF_TEST_REPORT("Invalid getEncoderSpeeds data");test_data_is_valid=true;}
        if (vel_mot==zero_vector) {RTF_TEST_REPORT("Invalid getMotorEncoderSpeeds data");test_data_is_valid=true;}
        if (acc_jnt==zero_vector) {RTF_TEST_REPORT("Invalid getEncoderAccelerations data");test_data_is_valid=true;}
        if (acc_mot==zero_vector) {RTF_TEST_REPORT("Invalid getMotorEncoderAccelerations data");test_data_is_valid=true;}

        enc_jnt2mot = matrix * enc_jnt;
        vel_jnt2mot = matrix * vel_jnt;
        acc_jnt2mot = matrix * acc_jnt;
        enc_jnt2mot = enc_jnt2mot * 100;
        vel_jnt2mot = vel_jnt2mot * 100;
        acc_jnt2mot = acc_jnt2mot * 100;

        bool reached= false;
        int in_position=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            double curr_val=0;
            if (go_to_max==false) curr_val = min[i];
            else                  curr_val = max[i];
            if (fabs(enc_jnt[i]-curr_val)<0.5) in_position++;
        }
        if (in_position==jointsList.size()) reached=true;

        if (elapsed >= 20.0)
        {
            RTF_ASSERT_ERROR("Timeout while moving joint");
        }

        if (reached)
        {
            if (go_to_max==false)
            {
                for (unsigned int i=0; i<jointsList.size(); i++)
                    ipos->positionMove(i,max[i]);
                go_to_max=true;
                cycle++;
                start_time = yarp::os::Time::now();
            }
            else
            {
                for (unsigned int i=0; i<jointsList.size(); i++)
                    ipos->positionMove(i,min[i]);
                go_to_max=false;
                cycle++;
                start_time = yarp::os::Time::now();
            }
        }

        //update previous and computes diff
        prev_enc_jnt     = enc_jnt;
        prev_enc_mot     = enc_mot;
        prev_enc_jnt2mot = enc_jnt2mot;
        prev_vel_jnt     = vel_jnt;
        prev_vel_mot     = vel_mot;
        prev_vel_jnt2mot = vel_jnt2mot;
        prev_acc_jnt     = acc_jnt;
        prev_acc_mot     = acc_mot;
        prev_acc_jnt2mot = acc_jnt2mot;
        diff_enc_jnt     = enc_jnt     - prev_enc_jnt;
        diff_enc_mot     = enc_mot     - prev_enc_mot;
        diff_enc_jnt2mot = enc_jnt2mot - prev_enc_jnt2mot;
        diff_vel_jnt     = vel_jnt     - prev_vel_jnt;
        diff_vel_mot     = vel_mot     - prev_vel_mot;
        diff_vel_jnt2mot = vel_jnt2mot - prev_vel_jnt2mot;
        diff_acc_jnt     = acc_jnt     - prev_acc_jnt;
        diff_acc_mot     = acc_mot     - prev_acc_mot;
        diff_acc_jnt2mot = acc_jnt2mot - prev_acc_jnt2mot;

        //prepare data to plot
        //JOINT POSITIONS vs MOTOR POSITIONS
        Bottle& row_test1 = dataToPlot_test1.addList();
        Bottle& v1_test1 = row_test1.addList();
        Bottle& v2_test1 = row_test1.addList();
        v1_test1.read(enc_mot);
        v2_test1.read(enc_jnt2mot);

        //JOINT VELOCITES vs MOTOR VELOCITIES
        Bottle& row_test2 = dataToPlot_test2.addList();
        Bottle& v1_test2 = row_test2.addList();
        Bottle& v2_test2 = row_test2.addList();
        v1_test2.read(vel_mot);
        v2_test2.read(vel_jnt2mot);

        //JOINT POSITIONS(DERIVED) vs JOINT SPEED
        Bottle& row_test3 = dataToPlot_test3.addList();
        Bottle& v1_test3 = row_test3.addList();
        Bottle& v2_test3 = row_test3.addList();
        v1_test3.read(vel_jnt);
        v2_test3.read(diff_enc_jnt2mot);

        //MOTOR POSITIONS(DERIVED) vs MOTOR SPEED
        Bottle& row_test4 = dataToPlot_test4.addList();
        Bottle& v1_test4 = row_test4.addList();
        Bottle& v2_test4 = row_test4.addList();
        v1_test4.read(vel_mot);
        v2_test4.read(diff_enc_mot);

        //exit condition
        if (cycle>=cycles) break;
    }

    goHome();

    string filename1 = "plot_test1.txt";
    saveToFile(filename1,dataToPlot_test1);
    string filename2 = "plot_test2.txt";
    saveToFile(filename2,dataToPlot_test2);
    string filename3 = "plot_test3.txt";
    saveToFile(filename3,dataToPlot_test3);
    string filename4 = "plot_test4.txt";
    saveToFile(filename4,dataToPlot_test4);

    char plotstring[2000];
    char temp[1000];

    sprintf (plotstring, "gnuplot -e \" unset key; set multiplot layout %d,1 title 'JOINT POSITIONS vs MOTOR POSITIONS'; ",jointsList.size());
    for (unsigned int col=0; col<jointsList.size(); col++)
    {sprintf (temp, " plot '%s' u %d with lines,  '%s' u %d with lines;", filename1.c_str(), col+1, filename1.c_str(), col+1+n_part_joints); strcat(plotstring,temp);}
    sprintf (temp, "unset multiplot; \" -persist "); strcat(plotstring,temp);
    system (plotstring);

    sprintf (plotstring, "gnuplot -e \" unset key; set multiplot layout %d,1 title 'JOINT VELOCITES vs MOTOR VELOCITIES' ; ",jointsList.size());
    for (unsigned int col=0; col<jointsList.size(); col++)
    {sprintf (temp, " plot '%s' u %d with lines,  '%s' u %d with lines;", filename2.c_str(), col+1, filename2.c_str(), col+1+n_part_joints); strcat(plotstring,temp);}
    sprintf (temp, "unset multiplot; \" -persist "); strcat(plotstring,temp);
    system (plotstring);

    sprintf (plotstring, "gnuplot -e \" unset key; set multiplot layout %d,1 title 'JOINT POSITIONS(DERIVED) vs JOINT SPEED ' ; ",jointsList.size());
    for (unsigned int col=0; col<jointsList.size(); col++)
    {sprintf (temp, " plot '%s' u %d with lines,  '%s' u %d with lines;", filename3.c_str(), col+1, filename3.c_str(), col+1+n_part_joints); strcat(plotstring,temp);}
    sprintf (temp, "unset multiplot; \" -persist "); strcat(plotstring,temp);
    system (plotstring);

    sprintf (plotstring, "gnuplot -e \" unset key; set multiplot layout %d,1 title 'JOINT POSITIONS(DERIVED) vs MOTOR SPEED ' ; ",jointsList.size());
    for (unsigned int col=0; col<jointsList.size(); col++)
    {sprintf (temp, " plot '%s' u %d with lines,  '%s' u %d with lines;", filename4.c_str(), col+1, filename4.c_str(), col+1+n_part_joints); strcat(plotstring,temp);}
    sprintf (temp, "unset multiplot; \" -persist "); strcat(plotstring,temp);
    system (plotstring);

    RTF_ASSERT_ERROR_IF(test_data_is_valid,"Invalid data obtained from encoders interface");
}
