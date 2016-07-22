// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <math.h>
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <rtf/yarp/YarpTestAsserter.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>
//#include <iostream>
//#include <yarp/manager/localbroker.h>
//#include <cstdlib>



#include <fstream>
#include <algorithm>    // std::replace


#include "TorqueControlStiffDumpCheck.h"


using namespace RTF;
using namespace RTF::YARP;
using namespace yarp::dev;
using namespace std;

// prepare the plugin
PREPARE_PLUGIN(TorqueControlStiffDumpCheck)

TorqueControlStiffDumpCheck::TorqueControlStiffDumpCheck() : YarpTestCase("TorqueControlStiffDumpCheck") {
    jointsList=0;
    dd=0;
    ipos=0;
    iamp=0;
    icmd=0;
    iimd=0;
    ienc=0;
    itrq=0;
    dumping=0;
    stiffness=0;
    home=0;
    n_part_joints=0;
    n_cmd_joints=0;
    plot_enabled = false;
}

TorqueControlStiffDumpCheck::~TorqueControlStiffDumpCheck() { }

bool TorqueControlStiffDumpCheck::setup(yarp::os::Property& property) {

    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    RTF_ASSERT_ERROR_IF(property.check("robot"),    "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("part"),     "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("joints"),   "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("home"),     "The home position list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("stiffness"),"The stiffness list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("dumping"),  "The dumping listmust be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(property.check("duration"), "The duration must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();



    Bottle* jointsBottle = property.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");
    n_cmd_joints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(n_cmd_joints>0,"invalid number of joints, it must be >0");

    Bottle *b_stiff = property.find("stiffness").asList();
    RTF_ASSERT_ERROR_IF(b_stiff!=0,"unable to parse stiffness parameter");
    RTF_ASSERT_ERROR_IF((b_stiff->size()==n_cmd_joints), Asserter::format("invalid number of stiffness values %d %d", b_stiff->size(), n_cmd_joints));

    Bottle *b_dump = property.find("dumping").asList();
    RTF_ASSERT_ERROR_IF(b_dump!=0,"unable to parse dumping parameter");
    RTF_ASSERT_ERROR_IF(b_dump->size()==n_cmd_joints,"invalid number of dumping values");

    Bottle *b_home = property.find("home").asList();
    RTF_ASSERT_ERROR_IF(b_home!=0,"unable to parse home parameter");
    RTF_ASSERT_ERROR_IF(b_home->size()==n_cmd_joints,"invalid number of home values");

    testLen_sec = property.find("duration").asDouble();
    RTF_ASSERT_ERROR_IF(testLen_sec>0, "duretion should be bigger than 0");

    if(property.check("plot_enabled"))
    {
        plot_enabled = property.find("plot_enabled").asBool();
    }
    if(plot_enabled)
        yInfo() << "Plot is enabled: the test will run octave and plot test result ";
    else
        yInfo() << "Plot is not enabled. The test collects only data. The user need to plot data to theck if test has successed.";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/TorqueControlStiffDumpCheckTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(itrq),"Unable to open torque control interface");
    RTF_ASSERT_ERROR_IF(dd->view(ienc),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(iamp),"Unable to open ampliefier interface");
    RTF_ASSERT_ERROR_IF(dd->view(ipos),"Unable to open position interface");
    RTF_ASSERT_ERROR_IF(dd->view(icmd),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimd),"Unable to open interaction mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iimp),"Unable to open impedence control interface");


    if (!ienc->getAxes(&n_part_joints))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    if(n_part_joints<=0)
        RTF_ASSERT_ERROR("Error this part has in invalid (<=0) number of jonits");



    jointsList=new int[n_cmd_joints];
    stiffness=new double[n_cmd_joints];
    dumping=new double[n_cmd_joints];
    pos_tot=new double[n_cmd_joints];

    for (int i=0; i <n_cmd_joints; i++) jointsList[i]=jointsBottle->get(i).asInt();
    for (int i=0; i <n_cmd_joints; i++) stiffness[i]=b_stiff->get(i).asDouble();
    for (int i=0; i <n_cmd_joints; i++) dumping[i]=b_dump->get(i).asDouble();
    return true;
}

void TorqueControlStiffDumpCheck::tearDown()
{
    if (jointsList) {delete jointsList; jointsList =0;}
    if (stiffness) {delete stiffness; stiffness =0;}
    if (dumping) {delete dumping; dumping =0;}
    if (dd) {delete dd; dd =0;}
}

void TorqueControlStiffDumpCheck::setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode)
{
    for (int i=0; i<n_cmd_joints; i++)
    {
        icmd->setControlMode(jointsList[i],desired_control_mode);
        iimd->setInteractionMode(jointsList[i],desired_interaction_mode);
        yarp::os::Time::delay(0.010);
    }
}

void TorqueControlStiffDumpCheck::verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, yarp::os::ConstString title)
{
    int cmode;
    yarp::dev::InteractionModeEnum imode;
    int timeout = 0;

    while (1)
    {
        int ok=0;
        for (int i=0; i<n_cmd_joints; i++)
        {
            icmd->getControlMode (jointsList[i],&cmode);
            iimd->getInteractionMode(jointsList[i],&imode);
            if (cmode==desired_control_mode && imode==desired_interaction_mode) ok++;
        }
        if (ok==n_cmd_joints) break;
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

void TorqueControlStiffDumpCheck::goHome()
{
    for (int i=0; i<n_cmd_joints; i++)
    {
        ipos->setRefSpeed(jointsList[i],20.0);
        ipos->positionMove(jointsList[i],home[i]);
    }

    int timeout = 0;
    while (1)
    {
        int in_position=0;
        for (int i=0; i<n_cmd_joints; i++)
        {
            ienc->getEncoder(jointsList[i],&pos_tot[jointsList[i]]);
            if (fabs(pos_tot[jointsList[i]]-home[i])<0.5) in_position++;
        }
        if (in_position==n_cmd_joints) break;
        if (timeout>100)
        {
            RTF_ASSERT_ERROR("Timeout while reaching home[i] position");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

bool TorqueControlStiffDumpCheck::setAndCheckImpedance(int joint, double stiffness, double dumping)
{
    iimp->setImpedance(joint, stiffness, dumping);
    double readStiff, readDump;
    iimp->getImpedance(joint, &readStiff, &readDump);

    double th = stiffness/100;
    bool r1 = YarpTestAsserter::isApproxEqual(readStiff, stiffness, th, th);
    th = dumping/100;
    bool r2 = YarpTestAsserter::isApproxEqual(readDump, dumping, th, th);

    if(!r1 || !r2)
    {
        RTF_TEST_REPORT(Asserter::format("J %d: set stiff=%f dump=%f. Read stiff=%f dump=%f ",joint, stiffness, dumping, readStiff, readDump));
        return false;
    }

    return true;
}

void TorqueControlStiffDumpCheck::saveToFile(std::string filename, yarp::os::Bottle &b)
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


std::string TorqueControlStiffDumpCheck::getPath(const std::string& str)
{
  size_t found;
  found=str.find_last_of("/\\");
  return(str.substr(0,found));
}


void TorqueControlStiffDumpCheck::run()
{
    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test0");

    goHome();

    setMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT,"test1");

    for (int i=0; i<n_cmd_joints; i++)
    {

        RTF_TEST_REPORT(Asserter::format("***** Start first part of test on joint %d......", jointsList[i]));
        RTF_ASSERT_ERROR_IF(setAndCheckImpedance(jointsList[i], stiffness[i], 0) , Asserter::format("Error setting impedance on j %d", jointsList[i]));

        RTF_TEST_REPORT("Now the user should move the joint....the test will collect values of position and torque. Press a char to continue....");
        char c;
        scanf("%c", &c);

        double start_time = yarp::os::Time::now();
        double curr_time = start_time;
        while(curr_time < start_time+testLen_sec)
        {
            double curr_pos, torque;
            ienc->getEncoder(jointsList[i], &curr_pos);
            itrq->getTorque(jointsList[i], &torque);

            Bottle& row = b_pos_trq.addList();
            row.addDouble(curr_pos-home[i]);
            row.addDouble(torque);
            yarp::os::Time::delay(0.01);
            curr_time = yarp::os::Time::now();
        }

        string testfilename = "posVStrq_";
        Bottle b;
        b.addInt(jointsList[i]);
        string filename1 = testfilename + partName + "_j" + b.toString().c_str() + ".txt";
        saveToFile(filename1,b_pos_trq);
        b_pos_trq.clear();


        RTF_TEST_REPORT(Asserter::format("....DONE on joint %d", jointsList[i]));
        RTF_TEST_REPORT(Asserter::format("***** Start second part of test on joint %d......", jointsList[i]));
        RTF_ASSERT_ERROR_IF(setAndCheckImpedance(jointsList[i], 0, dumping[i]) , Asserter::format("Error setting impedance on j %d", jointsList[i]));

        RTF_TEST_REPORT("Now the user should move the joint....the test will collect values of position and torque. Press a char to continue....");

        scanf("%c", &c);

        start_time = yarp::os::Time::now();
        curr_time = start_time;
        while(curr_time < start_time+testLen_sec)
        {
            double curr_vel, torque;
            ienc->getEncoderSpeed(jointsList[i], &curr_vel);
            itrq->getTorque(jointsList[i], &torque);

            Bottle& row = b_vel_trq.addList();
            row.addDouble(curr_vel);
            row.addDouble(torque);
            yarp::os::Time::delay(0.01);
            curr_time = yarp::os::Time::now();
        }

        testfilename = "velVStrq_";
        Bottle b1;
        b1.addInt(jointsList[i]);
        filename1 = testfilename + partName + "_j" + b1.toString().c_str() + ".txt";
        saveToFile(filename1,b_vel_trq);
        b_vel_trq.clear();
        RTF_TEST_REPORT(Asserter::format("....DONE on joint %d", jointsList[i]));

    }//end for

    RTF_TEST_REPORT("Test ended. Puts joints in pos stiff and moves them to home pos");
    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test2");

    goHome();

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("scripts");

    //find octave scripts
    std::string octaveFile = rf.findFile("torqueStiffDump_plotAll.m");
    if(octaveFile.size() == 0)
    {
        yError()<<"Cannot find file encoderConsistencyPlotAll.m";
        return;
    }

    //prepare octave command
    std::string octaveCommand= "octave --path "+ getPath(octaveFile);

    //transform stiffness, dumping and jointslist in a vector string for octave
    stringstream ss_stifness, ss_dumping, ss_joints;
    ss_stifness << "[";
    ss_dumping << "[";
    ss_joints << "[";
    for(int j=0; j<n_cmd_joints-1; j++)
    {
        ss_stifness << stiffness[j] <<", ";
        ss_dumping << dumping[j] <<", ";
        ss_joints << jointsList[j] <<", ";

    }
    ss_stifness << stiffness[n_cmd_joints-1] << "]";
    ss_dumping << dumping[n_cmd_joints-1] << "]";
    ss_joints << jointsList[n_cmd_joints-1] << "]";

    stringstream ss;
    ss << n_cmd_joints <<  ", "<< ss_stifness.str() << ", " << ss_dumping.str() << ", " << ss_joints.str();
    string str = ss.str();
    octaveCommand+= " -q --eval \"torqueStiffDump_plotAll('" +partName +"'," + str +")\"  --persist";

    yInfo() << "octave cmd= " << octaveCommand;
    if(plot_enabled)
    {
        int ret = system (octaveCommand.c_str());
    }
    else
    {
         yInfo() << "Test has collected all data. You need to plot data to check is test is passed";
         yInfo() << "Please run following command to plot data.";
         yInfo() << octaveCommand;
         yInfo() << "To exit from Octave application please type 'exit' command.";
    }
}


//void TorqueControlStiffDumpCheck::run()
//{
////    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
////   verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test0");

////    goHome();

////    setMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT);
////   verifyMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT,"test1");

//    for (int i=0; i<n_cmd_joints; i++)
//    {

//        RTF_TEST_REPORT(Asserter::format("***** Start first part of test on joint %d......", jointsList[i]));
//        //RTF_ASSERT_ERROR_IF(setAndCheckImpedance(jointsList[i], stiffness[i], 0) , Asserter::format("Error setting impedance on j %d", jointsList[i]));

//        RTF_TEST_REPORT("Now the user should move the joint....the test will collect values of position and torque. Press a char to continue....");
//        char c;
//        scanf("%c", &c);

//        double start_time = yarp::os::Time::now();
//        double curr_time = start_time;
//        int x=0;
//        while(curr_time < start_time+testLen_sec)
//        {
//            double curr_pos, torque;
//            //ienc->getEncoder(jointsList[i], curr_pos);
//            //itrq->getTorque(jointsList[i], torque);

//            Bottle& row = b_pos_trq.addList();
//            //row.addDouble(curr_pos-home[i], torque);
//            row.addDouble(x);
//            row.addDouble(10+x);
//            yarp::os::Time::delay(0.01);
//            curr_time = yarp::os::Time::now();
//            x++;
//        }

//        string testfilename = "posVStrq_";
//        Bottle b;
//        b.addInt(jointsList[i]);
//        string filename1 = testfilename + partName + "_j" + b.toString().c_str() + ".txt";
//        saveToFile(filename1,b_pos_trq);
//        b_pos_trq.clear();




//        RTF_TEST_REPORT(Asserter::format("***** Start second part of test on joint %d......", jointsList[i]));
//        RTF_ASSERT_ERROR_IF(setAndCheckImpedance(jointsList[i], 0, dumping[i]) , Asserter::format("Error setting impedance on j %d", jointsList[i]));

//        RTF_TEST_REPORT("Now the user should move the joint....the test will collect values of position and torque. Press a char to continue....");

//        scanf("%c", &c);
//        x=1000;
//        start_time = yarp::os::Time::now();
//        curr_time = start_time;
//        while(curr_time < start_time+testLen_sec)
//        {
//            double curr_vel, torque;
//            //ienc->getEncoderSpeed(jointsList[i], curr_vel);
//            //itrq->getTorque(jointsList[i], torque);

//            Bottle& row = b_vel_trq.addList();
//            row.addDouble(x);
//            row.addDouble(x+20);
//            yarp::os::Time::delay(0.01);
//            curr_time = yarp::os::Time::now();
//            x++;
//        }

//        testfilename = "velVStrq_";
//        Bottle b1;
//        b1.addInt(jointsList[i]);
//        filename1 = testfilename + partName + "_j" + b1.toString().c_str() + ".txt";
//        saveToFile(filename1,b_vel_trq);
//        b_vel_trq.clear();


//    }//end for

////    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
////    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test2");

////    goHome();

//    yarp::os::ResourceFinder rf;
//    rf.setDefaultContext("scripts");

//    //find octave scripts
//    std::string octaveFile = rf.findFile("torqueStiffDump_plotAll.m");
//    if(octaveFile.size() == 0)
//    {
//        yError()<<"Cannot find file encoderConsistencyPlotAll.m";
//        return;
//    }

//    //prepare octave command
//    std::string octaveCommand= "octave --path "+ getPath(octaveFile);

//    //transform stiffness, dumping and jointslist in a vector string for octave
//    stringstream ss_stifness, ss_dumping, ss_joints;
//    ss_stifness << "[";
//    ss_dumping << "[";
//    ss_joints << "[";
//    for(int j=0; j<n_cmd_joints-1; j++)
//    {
//        ss_stifness << stiffness[j] <<", ";
//        ss_dumping << dumping[j] <<", ";
//        ss_joints << jointsList[j] <<", ";

//    }
//    ss_stifness << stiffness[n_cmd_joints-1] << "]";
//    ss_dumping << dumping[n_cmd_joints-1] << "]";
//    ss_joints << jointsList[n_cmd_joints-1] << "]";

//    stringstream ss;
//    ss << n_cmd_joints <<  ", "<< ss_stifness.str() << ", " << ss_dumping.str() << ", " << ss_joints.str();
//    string str = ss.str();
//    octaveCommand+= " -q --eval \"torqueStiffDump_plotAll('" +partName +"'," + str +")\"  --persist";

//    yInfo() << "octave cmd= " << octaveCommand;
//    if(plot_enabled)
//    {
//        int ret = system (octaveCommand.c_str());
//    }
//    else
//    {
//         yInfo() << "Test has collected all data. You need to plot data to check is test is passed";
//         yInfo() << "Please run following command to plot data.";
//         yInfo() << octaveCommand;
//         yInfo() << "To exit from Octave application please type 'exit' command.";
//    }

//}
