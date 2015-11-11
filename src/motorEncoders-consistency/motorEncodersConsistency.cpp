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
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include "motorEncodersConsistency.h"
#include <yarp/manager/localbroker.h>
#include <iostream>

using namespace std;

//example     -v -t OpticalEncodersConsistency.dll -p "--robot icub --part left_arm --joints ""(0 1 2)"" --home ""(-30 30 10)"" --speed ""(20 20 20)"" --max ""(-20 40 20)"" --min ""(-40 20 0)"" --cycles 10 --tolerance 1.0 "
//example2    -v -t OpticalEncodersConsistency.dll -p "--robot icub --part head     --joints ""(2)""     --home ""(0)""         --speed ""(20      )"" --max ""(10      )""  --min ""(-10)""      --cycles 10 --tolerance 1.0 "
//-v - s "C:\software\icub-tests\suits\encoders-icubSim.xml"
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
    imotenc=0;
   
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
    position_move_tolerance = 1.0;
}

OpticalEncodersConsistency::~OpticalEncodersConsistency() { }

bool OpticalEncodersConsistency::setup(yarp::os::Property& property) {

    if(property.check("name"))
        setName(property.find("name").asString());

    char b[5000];
    strcpy (b,property.toString().c_str());
    RTF_TEST_REPORT("on setup()");
    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"), "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"), "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("home"),      "The home position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("max"),       "The max position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("min"),       "The min position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("speed"),     "The positionMove reference speed must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("tolerance"), "The tolerance of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("matrix_size"),  "The matrix size must be given!");
    RTF_ASSERT_ERROR_IF(property.check("matrix"),       "The coupling matrix must be given!");
    robotName = property.find("robot").asString();
    partName = property.find("part").asString();
    plotString1 = property.find("plotString1").asString();
    plotString2 = property.find("plotString2").asString();
    plotString3 = property.find("plotString3").asString();
    plotString4 = property.find("plotString4").asString();
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

    int matrix_size=property.find("matrix_size").asInt();
    if (matrix_size>0)
    {
        matrix.resize(matrix_size,matrix_size);
        matrix.eye();
        Bottle* matrixBottle = property.find("matrix").asList();
        if (matrixBottle!= NULL && matrixBottle->size() == (matrix_size*matrix_size) )
        {
            for (int i=0; i< (matrix_size*matrix_size); i++)
            {
                matrix.data()[i]=matrixBottle->get(i).asDouble();
            }
        }
        else
        {
           char buff [500];
           sprintf (buff, "invalid number of elements of parameter matrix %d!=%d", matrixBottle->size() , (matrix_size*matrix_size));
           RTF_ASSERT_ERROR(buff);
        }
    }
    else
    {
        RTF_ASSERT_ERROR("invalid matrix_size: must be >0");
    }

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
    RTF_ASSERT_ERROR_IF(dd->view(imotenc),"Unable to open motor encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(imot),"Unable to open motor interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
    jointsList.clear();
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    enc_jnt.resize(n_cmd_joints); enc_jnt.zero();
    enc_mot.resize(n_cmd_joints); enc_mot.zero();
    vel_jnt.resize(n_cmd_joints); vel_jnt.zero();
    vel_mot.resize(n_cmd_joints); vel_mot.zero();
    acc_jnt.resize(n_cmd_joints); acc_jnt.zero();
    acc_mot.resize(n_cmd_joints); acc_mot.zero();
    prev_enc_jnt.resize(n_cmd_joints); prev_enc_jnt.zero();
    prev_enc_mot.resize(n_cmd_joints); prev_enc_mot.zero();
    prev_enc_jnt2mot.resize(n_cmd_joints); prev_enc_jnt2mot.zero();
    prev_vel_jnt.resize(n_cmd_joints); prev_vel_jnt.zero();
    prev_vel_mot.resize(n_cmd_joints); prev_vel_mot.zero();
    prev_vel_jnt2mot.resize(n_cmd_joints); prev_vel_jnt2mot.zero();
    prev_acc_jnt.resize(n_cmd_joints); prev_acc_jnt.zero();
    prev_acc_mot.resize(n_cmd_joints); prev_acc_mot.zero();
    prev_acc_jnt2mot.resize(n_cmd_joints); prev_acc_jnt2mot.zero();
    zero_vector.resize(n_cmd_joints);
    zero_vector.zero(); 

    max.resize(n_cmd_joints);     for (int i=0; i< n_cmd_joints; i++) max[i]=maxBottle->get(i).asDouble();
    min.resize(n_cmd_joints);     for (int i=0; i< n_cmd_joints; i++) min[i]=minBottle->get(i).asDouble();
    home.resize(n_cmd_joints);    for (int i=0; i< n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    speed.resize(n_cmd_joints);   for (int i=0; i< n_cmd_joints; i++) speed[i]=speedBottle->get(i).asDouble();
    gearbox.resize(n_cmd_joints);
    for (int i=0; i< n_cmd_joints; i++)
    {
        double t;
        int b=imot->getGearboxRatio(jointsList[i],&t);
        gearbox[i]=t;
    }

    return true;
}

void OpticalEncodersConsistency::tearDown()
{
    char buff[500];
    sprintf(buff,"Closing test module");RTF_TEST_REPORT(buff);
    setMode(VOCAB_CM_POSITION);
    goHome();
    if (dd) {delete dd; dd =0;}
}

void OpticalEncodersConsistency::setMode(int desired_mode)
{
    if (icmd == 0) RTF_ASSERT_ERROR("Invalid control mode interface");
    if (iimd == 0) RTF_ASSERT_ERROR("Invalid interaction mode interface");

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
    if (ipos == 0) RTF_ASSERT_ERROR("Invalid position control interface");
    if (ienc == 0) RTF_ASSERT_ERROR("Invalid encoders interface");

    bool ret = true;
    char buff [500];
    sprintf(buff,"Homing the whole part");RTF_TEST_REPORT(buff);

    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ret = ipos->setRefSpeed((int)jointsList[i],speed[i]);
        RTF_ASSERT_ERROR_IF(ret, "ipos->setRefSpeed returned false");
        ret = ipos->positionMove((int)jointsList[i],home[i]);
        RTF_ASSERT_ERROR_IF(ret, "ipos->positionMove returned false");
    }

    int timeout = 0;
    while (1)
    {
        int in_position=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            double tmp=0;
            ienc->getEncoder((int)jointsList[i],&tmp);
            if (fabs(tmp-home[i])<position_move_tolerance) in_position++;
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
    char buff [500];
    setMode(VOCAB_CM_POSITION);
    goHome();

    bool go_to_max=false;
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipos->positionMove((int)jointsList[i], min[i]);
    }

    int  cycle=0;
    double start_time = yarp::os::Time::now();

    trasp_matrix = matrix.transposed();
    inv_matrix = yarp::math::luinv(matrix);
    inv_trasp_matrix = inv_matrix.transposed();

    sprintf(buff,"Matrix:\n %s \n", matrix.toString().c_str());
    RTF_TEST_REPORT(buff);
    sprintf(buff,"Inv matrix:\n %s \n", inv_matrix.toString().c_str());
    RTF_TEST_REPORT(buff);

    Bottle dataToPlot_test1;
    Bottle dataToPlot_test2;
    Bottle dataToPlot_test3;
    Bottle dataToPlot_test4;

    bool test_data_is_valid = false;
    bool first_time = true;
    yarp::sig::Vector off_enc_mot; off_enc_mot.resize(jointsList.size());
    yarp::sig::Vector off_enc_jnt2mot; off_enc_jnt2mot.resize(jointsList.size());
    yarp::sig::Vector tmp_vector;
    tmp_vector.resize(n_part_joints);

    while (1)
    {
        double curr_time = yarp::os::Time::now();
        double elapsed = curr_time - start_time;

        bool ret = true;
        ret = ienc->getEncoders(tmp_vector.data());
        for (unsigned int i = 0; i < jointsList.size(); i++)
            enc_jnt[i] = tmp_vector[jointsList[i]];


        RTF_ASSERT_ERROR_IF(ret, "ienc->getEncoders returned false");
        ret = imotenc->getMotorEncoders(tmp_vector.data());             for (unsigned int i = 0; i < jointsList.size(); i++) enc_mot[i] = tmp_vector[jointsList(i)];
        RTF_ASSERT_ERROR_IF(ret, "imotenc->getMotorEncoder returned false");
        ret = ienc->getEncoderSpeeds(tmp_vector.data());             for (unsigned int i = 0; i < jointsList.size(); i++) vel_jnt[i] = tmp_vector[jointsList(i)];
        RTF_ASSERT_ERROR_IF(ret, "ienc->getEncoderSpeeds returned false");
        ret = imotenc->getMotorEncoderSpeeds(tmp_vector.data());        for (unsigned int i = 0; i < jointsList.size(); i++) vel_mot[i] = tmp_vector[jointsList(i)];
        RTF_ASSERT_ERROR_IF(ret, "imotenc->getMotorEncoderSpeeds returned false");
        ret = ienc->getEncoderAccelerations(tmp_vector.data());      for (unsigned int i = 0; i < jointsList.size(); i++) acc_jnt[i] = tmp_vector[jointsList(i)];
        RTF_ASSERT_ERROR_IF(ret, "ienc->getEncoderAccelerations returned false");
        ret = imotenc->getMotorEncoderAccelerations(tmp_vector.data()); for (unsigned int i = 0; i < jointsList.size(); i++) acc_mot[i] = tmp_vector[jointsList(i)];
        RTF_ASSERT_ERROR_IF(ret, "imotenc->getMotorEncoderAccelerations returned false");

        if (enc_jnt == zero_vector) { RTF_TEST_REPORT("Invalid getEncoders data"); test_data_is_valid = true; }
        if (enc_mot == zero_vector) { RTF_TEST_REPORT("Invalid getMotorEncoders data"); test_data_is_valid = true; }
        if (vel_jnt == zero_vector) { RTF_TEST_REPORT("Invalid getEncoderSpeeds data"); test_data_is_valid = true; }
        if (vel_mot == zero_vector) { RTF_TEST_REPORT("Invalid getMotorEncoderSpeeds data"); test_data_is_valid = true; }
        if (acc_jnt == zero_vector) { RTF_TEST_REPORT("Invalid getEncoderAccelerations data"); test_data_is_valid = true; }
        if (acc_mot == zero_vector) { RTF_TEST_REPORT("Invalid getMotorEncoderAccelerations data"); test_data_is_valid = true; }

        enc_jnt2mot = matrix * enc_jnt;
        vel_jnt2mot = matrix * vel_jnt;
        acc_jnt2mot = matrix * acc_jnt;
        for (unsigned int i = 0; i < jointsList.size(); i++) enc_jnt2mot[i] = enc_jnt2mot[i] * gearbox[i];
        for (unsigned int i = 0; i < jointsList.size(); i++) vel_jnt2mot[i] = vel_jnt2mot[i] * gearbox[i];
        for (unsigned int i = 0; i < jointsList.size(); i++) acc_jnt2mot[i] = acc_jnt2mot[i] * gearbox[i];

        bool reached = false;
        int in_position = 0;
        for (unsigned int i = 0; i < jointsList.size(); i++)
        {
            double curr_val = 0;
            if (go_to_max == false) curr_val = min[i];
            else                  curr_val = max[i];
            if (fabs(enc_jnt[i] - curr_val) < position_move_tolerance) in_position++;
        }
        if (in_position == jointsList.size()) reached = true;

        if (elapsed >= 20.0)
        {
            RTF_ASSERT_ERROR("Timeout while moving joint");
        }

        if (reached)
        {
            sprintf(buff, "Test cycle %d/%d", cycle, cycles); RTF_TEST_REPORT(buff);
            if (go_to_max == false)
            {
                for (unsigned int i = 0; i < jointsList.size(); i++)
                    ipos->positionMove(jointsList[i], max[i]);
                go_to_max = true;
                cycle++;
                start_time = yarp::os::Time::now();
            }
            else
            {
                for (unsigned int i = 0; i < jointsList.size(); i++)
                    ipos->positionMove(jointsList[i], min[i]);
                go_to_max = false;
                cycle++;
                start_time = yarp::os::Time::now();
            }
        }

        //update previous and computes diff
        diff_enc_jnt = (enc_jnt - prev_enc_jnt) / 0.010;
        diff_enc_mot = (enc_mot - prev_enc_mot) / 0.010;
        diff_enc_jnt2mot = (enc_jnt2mot - prev_enc_jnt2mot) / 0.010;
        diff_vel_jnt = (vel_jnt - prev_vel_jnt) / 0.010;
        diff_vel_mot = (vel_mot - prev_vel_mot) / 0.010;
        diff_vel_jnt2mot = (vel_jnt2mot - prev_vel_jnt2mot) / 0.010;
        diff_acc_jnt = (acc_jnt - prev_acc_jnt) / 0.010;
        diff_acc_mot = (acc_mot - prev_acc_mot) / 0.010;
        diff_acc_jnt2mot = (acc_jnt2mot - prev_acc_jnt2mot) / 0.010;
        prev_enc_jnt = enc_jnt;
        prev_enc_mot = enc_mot;
        prev_enc_jnt2mot = enc_jnt2mot;
        prev_vel_jnt = vel_jnt;
        prev_vel_mot = vel_mot;
        prev_vel_jnt2mot = vel_jnt2mot;
        prev_acc_jnt = acc_jnt;
        prev_acc_mot = acc_mot;
        prev_acc_jnt2mot = acc_jnt2mot;

        if (first_time)
        {
            off_enc_mot = enc_mot;
            off_enc_jnt2mot = enc_jnt2mot;
        }

        //prepare data to plot
        //JOINT POSITIONS vs MOTOR POSITIONS
        Bottle& row_test1 = dataToPlot_test1.addList();
        Bottle& v1_test1 = row_test1.addList();
        Bottle& v2_test1 = row_test1.addList();
        yarp::sig::Vector v1 = enc_mot - off_enc_mot;
        yarp::sig::Vector v2 = enc_jnt2mot - off_enc_jnt2mot;
        v1_test1.read(v1);
        v2_test1.read(v2);

        //JOINT VELOCITES vs MOTOR VELOCITIES
        Bottle& row_test2 = dataToPlot_test2.addList();
        Bottle& v1_test2 = row_test2.addList();
        Bottle& v2_test2 = row_test2.addList();
        v1_test2.read(vel_mot);
        v2_test2.read(vel_jnt2mot);

        //JOINT POSITIONS(DERIVED) vs JOINT SPEED
        if (first_time == false)
        {
            Bottle& row_test3 = dataToPlot_test3.addList();
            Bottle& v1_test3 = row_test3.addList();
            Bottle& v2_test3 = row_test3.addList();
            v1_test3.read(vel_jnt);
            v2_test3.read(diff_enc_jnt);
        }

        //MOTOR POSITIONS(DERIVED) vs MOTOR SPEED
        if (first_time == false)
        {
            Bottle& row_test4 = dataToPlot_test4.addList();
            Bottle& v1_test4 = row_test4.addList();
            Bottle& v2_test4 = row_test4.addList();
            v1_test4.read(vel_mot);
            v2_test4.read(diff_enc_mot);
        }

        first_time = false;

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

    system(plotString1.c_str());
    system(plotString2.c_str());
    system(plotString3.c_str());
    system(plotString4.c_str());

    RTF_ASSERT_ERROR_IF(test_data_is_valid,"Invalid data obtained from encoders interface");
}
