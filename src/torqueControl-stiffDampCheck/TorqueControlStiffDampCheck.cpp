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

#include <math.h>
#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>
#include <yarp/robottestingframework/TestAsserter.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
//#include <iostream>
//#include <yarp/manager/localbroker.h>
//#include <cstdlib>



#include <fstream>
#include <algorithm>    // std::replace


#include "TorqueControlStiffDampCheck.h"


using namespace robottestingframework;

using namespace yarp::dev;
using namespace std;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TorqueControlStiffDampCheck)

TorqueControlStiffDampCheck::TorqueControlStiffDampCheck() : yarp::robottestingframework::TestCase("TorqueControlStiffDampCheck") {
    jointsList=0;
    dd=0;
    ipos=0;
    iamp=0;
    icmd=0;
    iimd=0;
    ienc=0;
    itrq=0;
    damping=0;
    stiffness=0;
    home=0;
    n_part_joints=0;
    n_cmd_joints=0;
    plot_enabled = false;
}

TorqueControlStiffDampCheck::~TorqueControlStiffDampCheck() { }

bool TorqueControlStiffDampCheck::setup(yarp::os::Property& property) {

    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    // updating parameters
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("robot"),    "The robot name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("part"),     "The part name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("joints"),   "The joints list must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("home"),     "The home position list must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("stiffness"),"The stiffness list must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("damping"),  "The damping listmust be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(property.check("duration"), "The duration must be given as the test parameter!");

    robotName = property.find("robot").asString();
    partName = property.find("part").asString();



    Bottle* jointsBottle = property.find("joints").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(jointsBottle!=0,"unable to parse joints parameter");
    n_cmd_joints = jointsBottle->size();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(n_cmd_joints>0,"invalid number of joints, it must be >0");

    Bottle *b_stiff = property.find("stiffness").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(b_stiff!=0,"unable to parse stiffness parameter");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE((b_stiff->size()==n_cmd_joints), Asserter::format("invalid number of stiffness values %d %d", b_stiff->size(), n_cmd_joints));

    Bottle *b_dump = property.find("damping").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(b_dump!=0,"unable to parse damping parameter");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(b_dump->size()==n_cmd_joints,"invalid number of damping values");

    Bottle *b_home = property.find("home").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(b_home!=0,"unable to parse home parameter");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(b_home->size()==n_cmd_joints,"invalid number of home values");

    testLen_sec = property.find("duration").asDouble();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(testLen_sec>0, "duretion should be bigger than 0");

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
    options.put("local", "/TorqueControlStiffDampCheckTest/"+robotName+"/"+partName);

    dd = new PolyDriver(options);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->isValid(),"Unable to open device driver");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(itrq),"Unable to open torque control interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ienc),"Unable to open encoders interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iamp),"Unable to open ampliefier interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(ipos),"Unable to open position interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(icmd),"Unable to open control mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iimd),"Unable to open interaction mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iimp),"Unable to open impedence control interface");


    if (!ienc->getAxes(&n_part_joints))
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    if(n_part_joints<=0)
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("Error this part has in invalid (<=0) number of jonits");



    jointsList=new int[n_cmd_joints];
    stiffness=new double[n_cmd_joints];
    damping=new double[n_cmd_joints];
    pos_tot=new double[n_cmd_joints];
    home=new double[n_cmd_joints];

    for (int i=0; i <n_cmd_joints; i++) jointsList[i]=jointsBottle->get(i).asInt();
    for (int i=0; i <n_cmd_joints; i++) stiffness[i]=b_stiff->get(i).asDouble();
    for (int i=0; i <n_cmd_joints; i++) damping[i]=b_dump->get(i).asDouble();
    for (int i=0; i <n_cmd_joints; i++) home[i]=b_home->get(i).asDouble();
    return true;
}

void TorqueControlStiffDampCheck::tearDown()
{
    if (jointsList) {delete jointsList; jointsList =0;}
    if (stiffness) {delete stiffness; stiffness =0;}
    if (damping) {delete damping; damping =0;}
    if (dd) {delete dd; dd =0;}
}

void TorqueControlStiffDampCheck::setMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode)
{
    for (int i=0; i<n_cmd_joints; i++)
    {
        icmd->setControlMode(jointsList[i],desired_control_mode);
        iimd->setInteractionMode(jointsList[i],desired_interaction_mode);
        yarp::os::Time::delay(0.010);
    }
}

void TorqueControlStiffDampCheck::verifyMode(int desired_control_mode, yarp::dev::InteractionModeEnum desired_interaction_mode, std::string title)
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
            char sbuf[800];
            sprintf(sbuf,"Test (%s) failed: current mode is (%s,%s), it should be (%s,%s)",title.c_str(),
                    Vocab::decode((NetInt32)desired_control_mode).c_str(),
                    Vocab::decode((NetInt32)desired_interaction_mode).c_str(),
                    Vocab::decode((NetInt32)cmode).c_str(),
                    Vocab::decode((NetInt32)imode).c_str());
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR(sbuf);
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
    char sbuf[500];
    sprintf(sbuf,"Test (%s) passed: current mode is (%s,%s)",title.c_str(), Vocab::decode((NetInt32)desired_control_mode).c_str(),
            Vocab::decode((NetInt32)desired_interaction_mode).c_str());
    ROBOTTESTINGFRAMEWORK_TEST_REPORT(sbuf);
}

void TorqueControlStiffDampCheck::goHome()
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
            ienc->getEncoder(jointsList[i],&pos_tot[i]);
            if (fabs(pos_tot[i]-home[i])<0.8) in_position++;
        }
        if (in_position==n_cmd_joints) break;
        if (timeout>100)
        {
            ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("Timeout while reaching home[i] position");
        }
        yarp::os::Time::delay(0.2);
        timeout++;
    }
}

bool TorqueControlStiffDampCheck::setAndCheckImpedance(int joint, double stiffness, double damping)
{
    iimp->setImpedance(joint, stiffness, damping);
    yarp::os::Time::delay(0.01);
    double readStiff, readDump;
    iimp->getImpedance(joint, &readStiff, &readDump);

    double th = stiffness/100;
    bool r1 = yarp::robottestingframework::TestAsserter::isApproxEqual(readStiff, stiffness, th, th);
    th = damping/100;
    bool r2 = yarp::robottestingframework::TestAsserter::isApproxEqual(readDump, damping, th, th);

    if(!r1 || !r2)
    {
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("J %d: set stiff=%f dump=%f. Read stiff=%f dump=%f ",joint, stiffness, damping, readStiff, readDump));
        return false;
    }

    return true;
}

void TorqueControlStiffDampCheck::saveToFile(std::string filename, yarp::os::Bottle &b)
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


std::string TorqueControlStiffDampCheck::getPath(const std::string& str)
{
  size_t found;
  found=str.find_last_of("/\\");
  return(str.substr(0,found));
}


void TorqueControlStiffDampCheck::run()
{
    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test0");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("try to move joints in home positions"));
    goHome();

    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("try to set compliant mode"));
    setMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT,"test1");

    for (int i=0; i<n_cmd_joints; i++)
    {

        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("***** Start first part of test on joint %d......", jointsList[i]));
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(setAndCheckImpedance(jointsList[i], stiffness[i], 0) , Asserter::format("Error setting impedance on j %d", jointsList[i]));



        //get initila torque
        double init_torque;
        itrq->getTorque(jointsList[i], &init_torque);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Now the user should move the joint....the test will collect values of position and torque. Press a char to continue....");
        char c;
        int unused = scanf("%c", &c);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("startingto collact data of joint %d......", jointsList[i]));

        double start_time = yarp::os::Time::now();
        double curr_time = start_time;
        while(curr_time < start_time+testLen_sec)
        {
            double curr_pos, torque, reftrq;
            ienc->getEncoder(jointsList[i], &curr_pos);
            itrq->getTorque(jointsList[i], &torque);
            itrq->getRefTorque(jointsList[i], &reftrq);

            Bottle& row = b_pos_trq.addList();
            row.addDouble(curr_pos-home[i]);
            row.addDouble(torque- init_torque);
            row.addDouble(reftrq);
            yarp::os::Time::delay(0.01);
            curr_time = yarp::os::Time::now();
        }

        string testfilename = "posVStrq_";
        Bottle b;
        b.addInt(jointsList[i]);
        string filename1 = testfilename + partName + "_j" + b.toString().c_str() + ".txt";
        saveToFile(filename1,b_pos_trq);
        b_pos_trq.clear();


        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("....DONE on joint %d", jointsList[i]));
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("***** Start second part of test on joint %d......", jointsList[i]));
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(setAndCheckImpedance(jointsList[i], 0, damping[i]) , Asserter::format("Error setting impedance on j %d", jointsList[i]));

        itrq->getTorque(jointsList[i], &init_torque);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Now the user should move the joint....the test will collect values of position and torque. Press a char to continue....");

        unused = scanf("%c", &c);

        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("startingto collact data of joint %d......", jointsList[i]));

        start_time = yarp::os::Time::now();
        curr_time = start_time;
        while(curr_time < start_time+testLen_sec)
        {
            double curr_vel, torque, reftrq;
            ienc->getEncoderSpeed(jointsList[i], &curr_vel);
            itrq->getTorque(jointsList[i], &torque);
            itrq->getRefTorque(jointsList[i], &reftrq);

            Bottle& row = b_vel_trq.addList();
            row.addDouble(curr_vel);
            row.addDouble(torque- init_torque);
            row.addDouble(reftrq);
            yarp::os::Time::delay(0.01);
            curr_time = yarp::os::Time::now();
        }

        testfilename = "velVStrq_";
        Bottle b1;
        b1.addInt(jointsList[i]);
        filename1 = testfilename + partName + "_j" + b1.toString().c_str() + ".txt";
        saveToFile(filename1,b_vel_trq);
        b_vel_trq.clear();
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("....DONE on joint %d", jointsList[i]));

    }//end for

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Test ended. Puts joints in pos stiff and moves them to home pos");
    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
    verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test2");

    goHome();

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("scripts");

    //find octave scripts
    std::string octaveFile = rf.findFile("torqueStiffDamp_plotAll.m");
    if(octaveFile.size() == 0)
    {
        yError()<<"Cannot find file encoderConsistencyPlotAll.m";
        return;
    }

    //prepare octave command
    std::string octaveCommand= "octave --path "+ getPath(octaveFile);

    //transform stiffness, damping and jointslist in a vector string for octave
    stringstream ss_stifness, ss_damping, ss_joints;
    ss_stifness << "[";
    ss_damping << "[";
    ss_joints << "[";
    for(int j=0; j<n_cmd_joints-1; j++)
    {
        ss_stifness << stiffness[j] <<", ";
        ss_damping << damping[j] <<", ";
        ss_joints << jointsList[j] <<", ";

    }
    ss_stifness << stiffness[n_cmd_joints-1] << "]";
    ss_damping << damping[n_cmd_joints-1] << "]";
    ss_joints << jointsList[n_cmd_joints-1] << "]";

    stringstream ss;
    ss << n_cmd_joints <<  ", "<< ss_stifness.str() << ", " << ss_damping.str() << ", " << ss_joints.str();
    string str = ss.str();
    octaveCommand+= " -q --eval \"torqueStiffDamp_plotAll('" +partName +"'," + str +")\"  --persist";

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


//void TorqueControlStiffDampCheck::run()
//{
////    setMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF);
////   verifyMode(VOCAB_CM_POSITION,VOCAB_IM_STIFF,"test0");

////    goHome();

////    setMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT);
////   verifyMode(VOCAB_CM_POSITION,VOCAB_IM_COMPLIANT,"test1");

//    for (int i=0; i<n_cmd_joints; i++)
//    {

//        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("***** Start first part of test on joint %d......", jointsList[i]));
//        //ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(setAndCheckImpedance(jointsList[i], stiffness[i], 0) , Asserter::format("Error setting impedance on j %d", jointsList[i]));

//        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Now the user should move the joint....the test will collect values of position and torque. Press a char to continue....");
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




//        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("***** Start second part of test on joint %d......", jointsList[i]));
//        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(setAndCheckImpedance(jointsList[i], 0, damping[i]) , Asserter::format("Error setting impedance on j %d", jointsList[i]));

//        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Now the user should move the joint....the test will collect values of position and torque. Press a char to continue....");

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
//    std::string octaveFile = rf.findFile("torqueStiffDamp_plotAll.m");
//    if(octaveFile.size() == 0)
//    {
//        yError()<<"Cannot find file encoderConsistencyPlotAll.m";
//        return;
//    }

//    //prepare octave command
//    std::string octaveCommand= "octave --path "+ getPath(octaveFile);

//    //transform stiffness, damping and jointslist in a vector string for octave
//    stringstream ss_stifness, ss_damping, ss_joints;
//    ss_stifness << "[";
//    ss_damping << "[";
//    ss_joints << "[";
//    for(int j=0; j<n_cmd_joints-1; j++)
//    {
//        ss_stifness << stiffness[j] <<", ";
//        ss_damping << damping[j] <<", ";
//        ss_joints << jointsList[j] <<", ";

//    }
//    ss_stifness << stiffness[n_cmd_joints-1] << "]";
//    ss_damping << damping[n_cmd_joints-1] << "]";
//    ss_joints << jointsList[n_cmd_joints-1] << "]";

//    stringstream ss;
//    ss << n_cmd_joints <<  ", "<< ss_stifness.str() << ", " << ss_damping.str() << ", " << ss_joints.str();
//    string str = ss.str();
//    octaveCommand+= " -q --eval \"torqueStiffDamp_plotAll('" +partName +"'," + str +")\"  --persist";

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
