// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Alessandro Scalzo and Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <cmath>

#include <rtf/dll/Plugin.h>
#include <rtf/TestAssert.h>
#include <rtf/yarp/YarpTestAsserter.h>

#include "MotorTest.h"

using namespace std;
using namespace RTF;
using namespace RTF::YARP;
using namespace yarp::os;

// prepare the plugin
PREPARE_PLUGIN(MotorTest)

MotorTest::MotorTest() : YarpTestCase("MotorTest") {
}

MotorTest::~MotorTest() {
}

bool MotorTest::setup(yarp::os::Property &configuration) {

    // initialization goes here ...
    m_aTargetVal=NULL;
    m_aMaxErr=NULL;
    m_aMinErr=NULL;
    m_aRefVel=NULL;
    m_aRefAcc=NULL;
    m_aTimeout=NULL;
    m_aHome=NULL;
    iEncoders=NULL;
    iPosition=NULL;
    m_initialized=false;

    if(configuration.check("name"))
        setName(configuration.find("name").asString());

    RTF_ASSERT_ERROR_IF(configuration.check("portname"),
                        "Missing 'portname' parameter, cannot open device");
    m_portname = configuration.find("portname").asString();

    RTF_ASSERT_ERROR_IF(configuration.check("joints"),
                        "Missing 'joints' parameter, cannot open device");
    m_NumJoints=configuration.find("joints").asInt();

    RTF_ASSERT_ERROR_IF(configuration.check("target"),
                        "Missing 'target' parameter, cannot open device");
    yarp::os::Bottle bot=configuration.findGroup("target").tail();
    int n = m_NumJoints<bot.size()? m_NumJoints:bot.size();
    m_aTargetVal=new double[m_NumJoints];
    m_aHome=new double [m_NumJoints];
    for (int i=0; i<n; ++i) {
        m_aTargetVal[i]=bot.get(i).asDouble();
        m_aHome[i]=0.0;
    }

    RTF_ASSERT_ERROR_IF(configuration.check("min"),
                        "Missing 'min' parameter, cannot open device");
    bot = configuration.findGroup("min").tail();
    n = m_NumJoints<bot.size()?m_NumJoints:bot.size();
    m_aMinErr=new double[m_NumJoints];
    for (int i=0; i<n; ++i)
       m_aMinErr[i]=bot.get(i).asDouble();

    RTF_ASSERT_ERROR_IF(configuration.check("max"),
                        "Missing 'max' parameter, cannot open device");
    bot=configuration.findGroup("max").tail();
    n = m_NumJoints<bot.size()? m_NumJoints:bot.size();
    m_aMaxErr=new double[m_NumJoints];
    for (int i=0; i<n; ++i)
         m_aMaxErr[i]=bot.get(i).asDouble();

    RTF_ASSERT_ERROR_IF(configuration.check("refvel"),
                        "Missing 'max' parameter, cannot open device");
    bot = configuration.findGroup("refvel").tail();
    n = m_NumJoints<bot.size()?m_NumJoints:bot.size();
    m_aRefVel=new double[m_NumJoints];
    for (int i=0; i<n; ++i)
        m_aRefVel[i]=bot.get(i).asDouble();

    if(configuration.check("refacc")) {
        bot = configuration.findGroup("refacc").tail();
        n = m_NumJoints<bot.size()?m_NumJoints:bot.size();
        m_aRefAcc=new double[m_NumJoints];
        for (int i=0; i<n; ++i)
            m_aRefAcc[i]=bot.get(i).asDouble();
    }

    RTF_ASSERT_ERROR_IF(configuration.check("timeout"),
                        "Missing 'timeout' parameter, cannot open device");
    bot = configuration.findGroup("timeout").tail();
    n = m_NumJoints<bot.size()?m_NumJoints:bot.size();
    m_aTimeout = new double[m_NumJoints];
    for (int i=0; i<n; ++i)
        m_aTimeout[i]=bot.get(i).asDouble();

    // opening interfaces
    yarp::os::Property options;
    options.put("device","remote_controlboard");
    options.put("local",m_portname+"/client");
    options.put("remote",m_portname);

    RTF_ASSERT_ERROR_IF(m_driver.open(options),
                        "cannot open driver");

    RTF_ASSERT_ERROR_IF(m_driver.view(iEncoders), "cannot view iEncoder");
    RTF_ASSERT_ERROR_IF(m_driver.view(iPosition), "cannot view iPosition");
    RTF_ASSERT_ERROR_IF(m_driver.view(iPosition2), "cannot view iPosition2");

    return true;
}

void MotorTest::tearDown() {
    // finalization goes her ...
    if(iPosition) {
        RTF_TEST_REPORT("Homing robot");
        iPosition->positionMove(m_aHome);

        bool reached=false;
        double timeStart=yarp::os::Time::now();
        double timeNow=timeStart;
        while(timeNow<timeStart+m_aTimeout[0] && !reached) {
            iPosition->checkMotionDone(&reached);
            timeNow=yarp::os::Time::now();
            yarp::os::Time::delay(0.1);
        }
    }

    if (m_aTargetVal) delete [] m_aTargetVal;
    if (m_aMaxErr)    delete [] m_aMaxErr;
    if (m_aMinErr)    delete [] m_aMinErr;
    if (m_aRefVel)    delete [] m_aRefVel;
    if (m_aRefAcc)    delete [] m_aRefAcc;
    if (m_aTimeout)   delete [] m_aTimeout;
    if (m_aHome)      delete [] m_aHome;
}

void MotorTest::run() {

    int nJoints=0;
    bool doneAll=false;
    bool ret=false;

    RTF_TEST_REPORT("checking joints number");
    iEncoders->getAxes(&nJoints);
    RTF_TEST_FAIL_IF(m_NumJoints==nJoints, "expected number of joints is inconsistent");

    RTF_TEST_REPORT("Checking individual joints...");
    for (int joint=0; joint<m_NumJoints; ++joint) {
        RTF_TEST_REPORT(Asserter::format("Checking joint %d", joint));
        if (m_aRefAcc!=NULL) {
            RTF_TEST_FAIL_IF(iPosition->setRefAcceleration(joint, m_aRefAcc[joint]),
                Asserter::format("setting reference acceleration on joint %d", joint));
        }

        RTF_TEST_FAIL_IF(iPosition->setRefSpeed(joint, m_aRefVel[joint]),
            Asserter::format("setting reference speed on joint %d", joint));

        // wait some time
        double timeStart=yarp::os::Time::now();
        double timeNow=timeStart;
        bool read=false;

        RTF_TEST_REPORT("Checking encoders");
        while(timeNow<timeStart+m_aTimeout[joint] && !read) {
            // read encoders
            read=iEncoders->getEncoder(joint,m_aHome+joint);
            yarp::os::Time::delay(0.1);
        }
        RTF_TEST_FAIL_IF(read, "getEncoder() returned true");

        RTF_TEST_FAIL_IF(iPosition->positionMove(joint, m_aTargetVal[joint]),
            Asserter::format("moving joint %d to %.2lf", joint, m_aTargetVal[joint]));

        doneAll=false;
        ret=iPosition->checkMotionDone(joint, &doneAll);
        RTF_TEST_FAIL_IF(!doneAll&&ret, "checking checkMotionDone returns false after position move");

        RTF_TEST_REPORT(Asserter::format("Waiting timeout %.2lf", m_aTimeout[joint]));
        bool reached=false;
        while(timeNow<timeStart+m_aTimeout[joint] && !reached) {
            double pos;
            iEncoders->getEncoder(joint,&pos);
            reached = YarpTestAsserter::isApproxEqual(pos, m_aTargetVal[joint], m_aMinErr[joint], m_aMaxErr[joint]);
            timeNow=yarp::os::Time::now();
            yarp::os::Time::delay(0.1);
        }
        RTF_TEST_FAIL_IF(reached, "reached position");
    }

    //////// check multiple joints
    RTF_TEST_REPORT("Checking multiple joints...");
    if (m_aRefAcc!=NULL) {
        RTF_TEST_FAIL_IF(iPosition->setRefAccelerations(m_aRefAcc),
                "setting reference acceleration on all joints");
    }
    RTF_TEST_FAIL_IF(iPosition->setRefSpeeds(m_aRefVel),
            "setting reference speed on all joints");

    RTF_TEST_FAIL_IF(iPosition->positionMove(m_aHome),
            "moving all joints to home");

    doneAll=false;
    // make sure that checkMotionDone return false right after a movement
    ret=iPosition->checkMotionDone(&doneAll);
    RTF_TEST_FAIL_IF(!doneAll&&ret, "checking checkMotionDone returns false after position move");

    // wait some time
    double timeStart=yarp::os::Time::now();
    double timeNow=timeStart;

    double timeout=m_aTimeout[0];
    for(int j=0; j<m_NumJoints; j++)
    {
        if (timeout<m_aTimeout[j])
            timeout=m_aTimeout[j];
    }

    RTF_TEST_REPORT(Asserter::format("Waiting timeout %.2lf", timeout));
    bool reached=false;
    double *encoders;
    encoders=new double [m_NumJoints];
    while(timeNow<timeStart+timeout && !reached) {
            RTF_TEST_FAIL_IF(iEncoders->getEncoders(encoders), "getEncoders()");
            reached = YarpTestAsserter::isApproxEqual(encoders, m_aHome, m_aMinErr, m_aMaxErr, m_NumJoints);
            timeNow=yarp::os::Time::now();
            yarp::os::Time::delay(0.1);
    }

    RTF_TEST_FAIL_IF(reached, "reached position");

    if(reached) {
        // check checkMotionDone.
        // because the previous movement was approximate, the robot
        // could still be moving so we need to iterate a few times

        int times=10;
        bool doneAll=false;
        bool ret=false;

        while(times>0 && !doneAll) {
            ret=iPosition->checkMotionDone(&doneAll);
            if (!doneAll)
                yarp::os::Time::delay(0.1);
            times--;
        }

        RTF_TEST_FAIL_IF(doneAll&&ret, "checking checkMotionDone returns true");
    }


    RTF_TEST_REPORT("Now checking group interface");

    //shuffle encoders
    int *jmap=new int [m_NumJoints];
    double *swapped_refvel=new double [m_NumJoints];
    double *swapped_target=new double [m_NumJoints];

    for(int kk=0;kk<m_NumJoints;kk++)
    {
        swapped_refvel[kk]=m_aRefVel[m_NumJoints-kk-1];
        swapped_target[kk]=m_aTargetVal[m_NumJoints-kk-1];
        jmap[kk]=m_NumJoints-kk-1;
    }

    RTF_TEST_FAIL_IF(iPosition2->setRefSpeeds(m_NumJoints, jmap, swapped_refvel),
            "setting reference speed on all joints using group interface");

    RTF_TEST_FAIL_IF(iPosition2->positionMove(m_NumJoints, jmap, swapped_target),
            "moving all joints to home using group interface");

    ret=iPosition2->checkMotionDone(m_NumJoints, jmap, &doneAll);
    RTF_TEST_FAIL_IF(!doneAll&&ret, "checking checkMotionDone returns false after position move");

    timeStart=yarp::os::Time::now();
    timeNow=timeStart;

    RTF_TEST_REPORT(Asserter::format("Waiting timeout %.2lf", timeout));
    reached=false;
    while(timeNow<timeStart+timeout && !reached) {
            iEncoders->getEncoders(encoders);
            reached = YarpTestAsserter::isApproxEqual(encoders, m_aTargetVal, m_aMinErr, m_aMaxErr, m_NumJoints);
            timeNow=yarp::os::Time::now();
            yarp::os::Time::delay(0.1);
    }
    RTF_TEST_FAIL_IF(reached, "reached position");

    if (reached) {
        bool *done_vector=new bool [m_NumJoints];
        // check checkMotionDone.
        // because the previous movement was approximate, the robot
        // could still be moving so we need to iterate a few times
        int times=10;
        bool doneAll=false;
        bool ret=false;

        while(times>0 && !doneAll) {
            ret=iPosition2->checkMotionDone(m_NumJoints, jmap, &doneAll);
            if (!doneAll)
                yarp::os::Time::delay(0.1);
            times--;
        }

        RTF_TEST_FAIL_IF(doneAll&&ret, "checking checkMotionDone");
        delete [] done_vector;
    }

    //cleanup
    delete [] jmap;
    delete [] swapped_refvel;
    delete [] swapped_target;
    delete [] encoders;
}
