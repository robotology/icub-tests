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
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include "opticalEncodersDrift.h"
#include <yarp/manager/localbroker.h>

//example     -v -t OpticalEncodersDrift.dll -p "--robot icub --part head --joints ""(0 1 2)"" --home ""(0 0 0)" --speed "(20 20 20)" --max "(10 10 10)" --min "(-10 -10 -10)" --cycles 100 --tolerance 1.0 "
//example2    -v -t OpticalEncodersDrift.dll -p "--robot icub --part head --joints ""(2)""     --home ""(0)""    --speed "(20      )" --max "(10      )" --min "(-10)"         --cycles 100 --tolerance 1.0 "
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

// prepare the plugin
PREPARE_PLUGIN(OpticalEncodersDrift)

OpticalEncodersDrift::OpticalEncodersDrift() : yarp::rtf::TestCase("OpticalEncodersDrift") {
    jointsList=0;
    dd=0;
    ipos=0;
    icmd=0;
    iimd=0;
    ienc=0;
    imot=0;
    enc_jnt=0;
    enc_mot=0;
    home_enc_mot=0;
    end_enc_mot=0;
    err_enc_mot=0;
    cycles=100;
}

OpticalEncodersDrift::~OpticalEncodersDrift() { }

bool OpticalEncodersDrift::setup(yarp::os::Property& property) {

    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    RTF_ASSERT_ERROR_IF_FALSE(property.check("robot"),     "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("part"),      "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("joints"),    "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("home"),      "The home position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("max"),       "The max position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("min"),       "The min position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("speed"),     "The positionMove reference speed must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("cycles"),    "The number of cycles of the control signal must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(property.check("tolerance"), "The max error tolerance must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF_FALSE(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* homeBottle = property.find("home").asList();
    RTF_ASSERT_ERROR_IF_FALSE(homeBottle!=0,"unable to parse zero parameter");

    Bottle* maxBottle = property.find("max").asList();
    RTF_ASSERT_ERROR_IF_FALSE(maxBottle!=0,"unable to parse max parameter");

    Bottle* minBottle = property.find("min").asList();
    RTF_ASSERT_ERROR_IF_FALSE(minBottle!=0,"unable to parse min parameter");

    Bottle* speedBottle = property.find("speed").asList();
    RTF_ASSERT_ERROR_IF_FALSE(speedBottle!=0,"unable to parse speed parameter");

    tolerance = property.find("tolerance").asDouble();
    RTF_ASSERT_ERROR_IF_FALSE(tolerance>=0,"invalid tolerance");

    cycles = property.find("cycles").asInt();
    RTF_ASSERT_ERROR_IF_FALSE(cycles>=0,"invalid cycles");

    if(property.check("plot_enabled"))
        plot = property.find("plot").asBool();
    else
        plot = true;

    if(plot)
        RTF_TEST_REPORT("This test will run gnuplot utility at the end.");
    else
        RTF_TEST_REPORT("This test will NOT run gnuplot utility at the end.");



    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/opticalEncodersDrift/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF_FALSE(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(iimd),"Unable to open interaction mode interface");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(imot),"Unable to open motor encoders interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF_FALSE(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    enc_jnt.resize(n_part_joints);
    enc_mot.resize(n_part_joints);
    home_enc_mot.resize(n_part_joints);
    end_enc_mot.resize(n_part_joints);
    err_enc_mot.resize(n_part_joints);

    max.resize  (n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) max[i]=maxBottle->get(i).asDouble();
    min.resize  (n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) min[i]=minBottle->get(i).asDouble();
    home.resize (n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    speed.resize(n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) speed[i]=speedBottle->get(i).asDouble();

    return true;
}

void OpticalEncodersDrift::tearDown()
{
    if (dd) {delete dd; dd =0;}
}

void OpticalEncodersDrift::setMode(int desired_mode)
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

bool OpticalEncodersDrift::goHome()
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
            if (fabs(tmp-home[i])<tolerance) in_position++;
        }
        if (in_position==jointsList.size()) break;
        if (timeout>100)
        {
            RTF_TEST_REPORT("Timeout while reaching home position");
            return false;
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
    return true;
}

void OpticalEncodersDrift::saveToFile(std::string filename, yarp::os::Bottle &b)
{
    std::fstream fs;
    fs.open (filename.c_str(), std::fstream::out);
    
    for (unsigned int i=0; i<b.size(); i++)
    {
        std::string s = b.get(i).toString();
        std::replace(s.begin(), s.end(), '(', ' ');
        std::replace(s.begin(), s.end(), ')', ' ');
        fs << s << endl;
    }

    fs.close();
}

void OpticalEncodersDrift::run()
{
    setMode(VOCAB_CM_POSITION);
    RTF_ASSERT_FAIL_IF_FALSE(goHome(), "Test can't run");

    bool go_to_max=false;
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipos->positionMove((int)jointsList[i], min[i]);
    }

    int  curr_cycle=0;
    double start_time = yarp::os::Time::now();
    Bottle dataToPlot;

    imot->getMotorEncoders             (home_enc_mot.data());
    while(1)
    {
        double curr_time = yarp::os::Time::now();
        double elapsed = curr_time-start_time;

        ienc->getEncoders                  (enc_jnt.data());
        imot->getMotorEncoders             (enc_mot.data());
        Bottle& row = dataToPlot.addList();
        Bottle& v1 = row.addList();
        Bottle& v2 = row.addList();
        v1.read(enc_jnt);
        v2.read(enc_mot);

        bool reached= false;
        int in_position=0;
        for (unsigned int i=0; i<jointsList.size(); i++)
        {
            double curr_val=0;
            if (go_to_max==false) curr_val = min[i];
            else                  curr_val = max[i];
            if (fabs(enc_jnt[i]-curr_val)<tolerance) in_position++;
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
                curr_cycle++;
                start_time = yarp::os::Time::now();
                if (curr_cycle % 10 == 0) RTF_TEST_REPORT(Asserter::format("Cycle %d/%d completed", curr_cycle, cycles));
            }
            else
            {
                for (unsigned int i=0; i<jointsList.size(); i++)
                    ipos->positionMove(i,min[i]);
                go_to_max=false;
                curr_cycle++;
                start_time = yarp::os::Time::now();
                if (curr_cycle % 10 == 0) RTF_TEST_REPORT(Asserter::format("Cycle %d/%d completed", curr_cycle, cycles));
            }
        }

        if (curr_cycle>=cycles) break;

        yarp::os::Time::delay(0.010);
    }

    bool isInHome = goHome();
    yarp::os::Time::delay(2.0);

    //automatic check, not complete yet
    {
        imot->getMotorEncoders             (end_enc_mot.data());
        for (int i=0; i<n_part_joints; i++)
        {
            err_enc_mot[i]=home_enc_mot[i]-end_enc_mot[i];

            if (fabs(err_enc_mot[i]) > tolerance)
            {
                //...assert something
            }
        }
    }

    
    string filename = "encDrift_plot_";
    filename += partName;
    filename += ".txt";

    int num_j = jointsList.size();
    saveToFile(filename,dataToPlot);

    char plotstring[1000];
    //gnuplot -e "unset key; plot for [col=1:6] 'C:\software\icub-tests\build\plugins\Debug\plot.txt' using col with lines" -persist
    sprintf (plotstring, "gnuplot -e \" unset key; plot for [col=1:%d] '%s' using col with lines \" -persist", num_j,filename.c_str());
    
    if(plot)
    {
        system (plotstring);
    }
    else
    {
        RTF_TEST_REPORT("Test is finished. Please check if collected date are ok, by using following command: ");
        RTF_TEST_REPORT(RTF::Asserter::format("%s", plotstring));
    }
    
    RTF_ASSERT_ERROR_IF_FALSE(isInHome, "This part is not in home. Suit test will be terminated!");

}
