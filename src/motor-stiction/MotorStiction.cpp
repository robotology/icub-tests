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
#include "MotorStiction.h"
#include <yarp/manager/localbroker.h>

//example1    -v -t MotorStiction.dll -p "--robot icub --part left_arm --joints ""(4)"" --home ""(45)"" --outputStep ""(20)"" --outputMax ""(3000)"" --outputDelay ""(2.0)""  --threshold ""(5.0)"" "

using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;

// prepare the plugin
PREPARE_PLUGIN(MotorStiction)

MotorStiction::MotorStiction() : yarp::rtf::TestCase("MotorStiction") {
    jointsList=0;
    dd=0;
    ipos=0;
    iamp=0;
    icmd=0;
    iimd=0;
    ienc=0;
    iopl=0;
}

MotorStiction::~MotorStiction() { }

bool MotorStiction::setup(yarp::os::Property& property) {
    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"),  "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"),   "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("home"),   "The home position must be given as the test parameter!");

    RTF_ASSERT_ERROR_IF(property.check("outputStep"),    "The output_step must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("outputDelay") ,  "The output_delay must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("outputMax"),     "The output_max must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("threshold"),     "The threshold must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("repeat"),        "The repeat must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();
    
    repeat = property.find("repeat").asInt();
    RTF_ASSERT_ERROR_IF(repeat>=0,"repeat must be greater than zero");

    Bottle* homeBottle = property.find("home").asList();
    RTF_ASSERT_ERROR_IF(homeBottle!=0,"unable to parse zero parameter");

    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* output_step_Bottle = property.find("outputStep").asList();
    RTF_ASSERT_ERROR_IF(output_step_Bottle!=0,"unable to parse joints parameter");

    Bottle* output_delay_Bottle = property.find("outputDelay").asList();
    RTF_ASSERT_ERROR_IF(output_delay_Bottle!=0,"unable to parse joints parameter");
    
    Bottle* output_max_Bottle = property.find("outputMax").asList();
    RTF_ASSERT_ERROR_IF(output_max_Bottle!=0,"unable to parse joints parameter");

    Bottle* threshold_Bottle = property.find("threshold").asList();
    RTF_ASSERT_ERROR_IF(threshold_Bottle!=0,"unable to parse joints parameter");

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/MotorStictionTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(iopl),"Unable to open openloop interface");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(iamp),"Unable to open ampliefier interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(ilim),"Unable to open limits interface");

    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    int n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0 && n_cmd_joints<=n_part_joints,"invalid number of joints, it must be >0 & <= number of part joints");
    for (int i=0; i <n_cmd_joints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    home.resize (n_cmd_joints);               for (int i=0; i< n_cmd_joints; i++) home[i]=homeBottle->get(i).asDouble();
    opl_step.resize (n_cmd_joints);           for (int i=0; i< n_cmd_joints; i++) opl_step[i]=output_step_Bottle->get(i).asDouble();
    opl_delay.resize (n_cmd_joints);          for (int i=0; i< n_cmd_joints; i++) opl_delay[i]=output_delay_Bottle->get(i).asDouble();
    opl_max.resize (n_cmd_joints);            for (int i=0; i< n_cmd_joints; i++) opl_max[i]=output_max_Bottle->get(i).asDouble();
    movement_threshold.resize (n_cmd_joints); for (int i=0; i< n_cmd_joints; i++) movement_threshold[i]=threshold_Bottle->get(i).asDouble();
    
    max_lims.resize(n_cmd_joints);
    min_lims.resize(n_cmd_joints);
    for (int i=0; i <n_cmd_joints; i++) ilim->getLimits((int)jointsList[i],&min_lims[i],&max_lims[i]);

    return true;
}

void MotorStiction::tearDown()
{
    char buff[500];
    sprintf(buff,"Closing test module");RTF_TEST_REPORT(buff);
    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test0");
    goHome();
    if (dd) {delete dd; dd =0;}
}

void MotorStiction::setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode)
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        icmd->setControlMode((int)jointsList[i],desired_control_mode);
        iimd->setInteractionMode((int)jointsList[i],desired_interaction_mode);
        yarp::os::Time::delay(0.010);
    }
}

void MotorStiction::setModeSingle(int i, int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode)
{
    icmd->setControlMode((int)jointsList[i],desired_control_mode);
    iimd->setInteractionMode((int)jointsList[i],desired_interaction_mode);
    yarp::os::Time::delay(0.010);
}

void MotorStiction::verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, yarp::os::ConstString title)
{
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
            if (cmode==desired_control_mode && imode==desired_interaction_mode) ok++;
        }
        if (ok==jointsList.size()) break;
        if (timeout>100)
        {
            char sbuf[500];
            sprintf(sbuf,"Test (%s) failed: current mode is (%d,%d), it should be (%d,%d)",title.c_str(), desired_control_mode,desired_interaction_mode,cmode,imode);
            RTF_ASSERT_ERROR(sbuf);
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
    char sbuf[500];
    sprintf(sbuf,"Test (%s) passed: current mode is (%d,%d)",title.c_str(), desired_control_mode,desired_interaction_mode);
    RTF_TEST_REPORT(sbuf);
}

void MotorStiction::goHome()
{
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        ipos->setRefSpeed((int)jointsList[i],20.0);
        ipos->positionMove((int)jointsList[i],home[i]);
        yarp::os::Time::delay(0.010);
    }

    char buff [500];
    sprintf(buff,"Homing the whole part");RTF_TEST_REPORT(buff);

    int in_position=0;
    for (unsigned int i=0; i<jointsList.size(); i++)
    {
        double time_started = yarp::os::Time::now();
        while (1)
        {
            double pos;
            ienc->getEncoder((int)jointsList[i],&pos);
            if (fabs(pos-home[i])<1.0) break;
            if (yarp::os::Time::now()-time_started>20) 
            {
                sprintf(buff,"Timeout while reaching zero position, joint %d, curr_enc %f, home %f", (int)jointsList[i],pos,home[i]);
                RTF_ASSERT_ERROR(buff);
            }
        }
    }

    sprintf(buff,"Homing succesfully completed");RTF_TEST_REPORT(buff);
}

void MotorStiction::saveToFile(std::string filename, yarp::os::Bottle &b)
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

void MotorStiction::OplExecute(int i, std::vector<yarp::os::Bottle>& dataToPlotList, stiction_data& current_test, bool positive_sign)
{
    char buff[500];
    double time     = yarp::os::Time::now();
    double time_old = yarp::os::Time::now();
    double enc=0;
    double start_enc=0;
    ienc->getEncoder((int)jointsList[i],&enc);
    ienc->getEncoder((int)jointsList[i],&start_enc);
    bool not_moving = true;
    double opl=0;
    setMode(VOCAB_CM_OPENLOOP,VOCAB_IM_STIFF);
    iopl->setRefOutput((int)jointsList[i],opl);
    double last_opl_cmd=yarp::os::Time::now();
    Bottle dataToPlot;

    while (not_moving)
    {
        Bottle& row = dataToPlot.addList();
        Bottle& v1 = row.addList();
        Bottle& v2 = row.addList();

        iopl->setRefOutput((int)jointsList[i],opl);
        ienc->getEncoder((int)jointsList[i],&enc);

        //sprintf(buff,"%f %f %f %f",enc,start_enc,fabs(enc-start_enc),movement_threshold[i]);RTF_TEST_REPORT(buff);

        if (fabs(enc-start_enc)>movement_threshold[i])
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            if (positive_sign) {current_test.pos_opl=opl; current_test.pos_test_passed=true;}
            else               {current_test.neg_opl=opl; current_test.neg_test_passed=true;}
            dataToPlotList.push_back(dataToPlot);
            sprintf(buff,"Test success (output=%f)",opl);RTF_TEST_REPORT(buff);
        }
        else if (opl>=opl_max[i])
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            if (positive_sign) {current_test.pos_opl=opl; current_test.pos_test_passed=false;}
            else               {current_test.neg_opl=opl; current_test.neg_test_passed=false;}
            dataToPlotList.push_back(dataToPlot);
            sprintf(buff,"Test failed failed because max output was reached(output=%f)",opl);RTF_TEST_REPORT(buff);
        }
        else if (fabs(enc-max_lims[i]) < 1.0 ||
                 fabs(enc-min_lims[i]) < 1.0 )
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            if (positive_sign) {current_test.pos_opl=opl; current_test.pos_test_passed=false;}
            else               {current_test.neg_opl=opl; current_test.neg_test_passed=false;}
            dataToPlotList.push_back(dataToPlot);
            sprintf(buff,"Test failed because hw limit was touched (enc=%f)",enc);RTF_TEST_REPORT(buff);
        }

        if (yarp::os::Time::now()-last_opl_cmd>opl_delay[i])
        {
            if (positive_sign)
            {opl+=opl_step[i];}
            else
            {opl-=opl_step[i];}
            last_opl_cmd=yarp::os::Time::now();
        }
            
        time = yarp::os::Time::now();
        v1.addDouble(time);
        v2.addDouble(enc);
        v2.addDouble(opl);
        yarp::os::Time::delay(0.010);

        if (time-time_old>5.0 && not_moving==true)
        {
            sprintf(buff,"test in progress on joint %d, current output value = %f",(int)jointsList[i],opl);RTF_TEST_REPORT(buff);
            time_old=time;
        }
    }
}

void MotorStiction::OplExecute2(int i, std::vector<yarp::os::Bottle>& dataToPlotList, stiction_data& current_test, bool positive_sign)
{
    char buff[500];
    double time     = yarp::os::Time::now();
    double time_old = yarp::os::Time::now();
    double enc=0;
    double start_enc=0;
    ienc->getEncoder((int)jointsList[i],&enc);
    ienc->getEncoder((int)jointsList[i],&start_enc);
    bool not_moving = true;
    double opl=0;
    setMode(VOCAB_CM_OPENLOOP,VOCAB_IM_STIFF);
    iopl->setRefOutput((int)jointsList[i],opl);
    double last_opl_cmd=yarp::os::Time::now();
    Bottle dataToPlot;

    while (not_moving)
    {
        Bottle& row = dataToPlot.addList();
        Bottle& v1 = row.addList();
        Bottle& v2 = row.addList();

        iopl->setRefOutput((int)jointsList[i],opl);
        ienc->getEncoder((int)jointsList[i],&enc);

        //sprintf(buff,"%f %f %f %f",enc,start_enc,fabs(enc-start_enc),movement_threshold[i]);RTF_TEST_REPORT(buff);

        if (opl>=opl_max[i])
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            if (positive_sign) {current_test.pos_opl=opl; current_test.pos_test_passed=false;}
            else               {current_test.neg_opl=opl; current_test.neg_test_passed=false;}
            dataToPlotList.push_back(dataToPlot);
            sprintf(buff,"Test failed failed because max output was reached(output=%f)",opl);RTF_TEST_REPORT(buff);
        }
        else if (fabs(enc-max_lims[i]) < 1.0 ||
                 fabs(enc-min_lims[i]) < 1.0 )
        {
            iopl->setRefOutput((int)jointsList[i],0.0);
            not_moving=false;
            if (positive_sign) {current_test.pos_opl=opl; current_test.pos_test_passed=true;}
            else               {current_test.neg_opl=opl; current_test.neg_test_passed=true;}
            dataToPlotList.push_back(dataToPlot);
            sprintf(buff,"Test success (output=%f)",opl);RTF_TEST_REPORT(buff);
        }

        if (yarp::os::Time::now()-last_opl_cmd>opl_delay[i])
        {
            if (positive_sign)
            {opl+=opl_step[i];}
            else
            {opl-=opl_step[i];}
            last_opl_cmd=yarp::os::Time::now();
        }
            
        time = yarp::os::Time::now();
        v1.addDouble(time);
        v2.addDouble(enc);
        v2.addDouble(opl);
        yarp::os::Time::delay(0.010);

        if (time-time_old>5.0 && not_moving==true)
        {
            sprintf(buff,"test in progress on joint %d, current output value = %f",(int)jointsList[i],opl);RTF_TEST_REPORT(buff);
            time_old=time;
        }
    }
}

void MotorStiction::run()
{
    //yarp::os::Time::delay(10);

    char buff[500];
    std::vector<Bottle> dataToPlotList;
    
    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    goHome();

    for (unsigned int i=0 ; i<jointsList.size(); i++)
    {
        for (int repeat_count=0; repeat_count<repeat; repeat_count++)
        {
            stiction_data current_test;
            current_test.jnt=(int)jointsList[i];
            current_test.cycle= repeat_count;

            setModeSingle(i,VOCAB_CM_OPENLOOP,VOCAB_IM_STIFF);
            iopl->setRefOutput((int)jointsList[i],0.0);

            sprintf(buff,"Testing joint %d, cycle %d, positive output",(int)jointsList[i],repeat_count);RTF_TEST_REPORT(buff);
            OplExecute(i,dataToPlotList,current_test, true);

            setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
            goHome();

            setModeSingle(i, VOCAB_CM_OPENLOOP,VOCAB_IM_STIFF);
            iopl->setRefOutput((int)jointsList[i],0.0);

            sprintf(buff,"Testing joint %d, cycle %d, negative output",(int)jointsList[i],repeat_count);RTF_TEST_REPORT(buff);
            OplExecute(i,dataToPlotList,current_test, false);

            setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
            goHome();

            //test cycle complete, save data
            stiction_data_list.push_back(current_test);

            char filename[500];
            sprintf (filename, "plot_stiction_%s_j%d_n_c%d.txt",partName.c_str(),(int)jointsList[i],repeat_count);
            saveToFile(filename,dataToPlotList.rbegin()[0]); //last element
            sprintf (filename, "plot_stiction_%s_j%d_p_c%d.txt",partName.c_str(),(int)jointsList[i],repeat_count);
            saveToFile(filename,dataToPlotList.rbegin()[1]); //second last element
        } 
    } 

    goHome();

    for (unsigned int i=0 ; i<jointsList.size(); i++)
    {
        char filename[500];
        char plotstring[1000];
        sprintf (filename, "plot_stiction_j%d.txt",(int)jointsList[i]);
        //gnuplot -e "unset key; plot for [col=1:6] 'C:\software\icub-tests\build\plugins\Debug\plot.txt' using col with lines" -persist
        sprintf (plotstring, "gnuplot -e \" unset key; plot for [col=1:%d] '%s' using col with lines \" -persist", n_part_joints,filename);
        //system (plotstring);
    }
    
    //stiction_data_list.size() include tests for all joints, multiple cycles
    for (unsigned int i=0; i <stiction_data_list.size(); i++)
    {
        if (stiction_data_list[i].neg_test_passed==false)
        {
            char buff [500];
            sprintf(buff, "test failed on joint %d, cycle %d, negative output value: %f",stiction_data_list[i].jnt,stiction_data_list[i].cycle,stiction_data_list[i].neg_opl);
            RTF_ASSERT_ERROR(buff);
        }
        else if (stiction_data_list[i].pos_test_passed==false)
        {
            char buff [500];
            sprintf(buff, "test failed on joint %d, cycle %d,  positive output value: %f",stiction_data_list[i].jnt,stiction_data_list[i].cycle,stiction_data_list[i].pos_opl);
            RTF_ASSERT_ERROR(buff);
        }
    }

}
