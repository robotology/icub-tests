// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <cmath>

#include <rtf/dll/Plugin.h>
#include <rtf/TestAssert.h>
#include <rtf/yarp/YarpTestAsserter.h>

#include "movementReferencesTest.h"

using namespace std;
using namespace RTF;
using namespace RTF::YARP;
using namespace yarp::os;


// prepare the plugin
PREPARE_PLUGIN(MovementReferencesTest)

MovementReferencesTest::MovementReferencesTest() : YarpTestCase("MovementReferencesTest") 
{
    targetPos=NULL;
    refVel=NULL;
    refAcc=NULL;
    homePos=NULL;

    numJointsInPart = 0;
    jPosMotion=NULL;

    iEncoders=NULL;
    iPosition2=NULL;
    iPosDirect=NULL;
    iControlMode2=NULL;
    iVelocity2=NULL;
    initialized=false;
}

MovementReferencesTest::~MovementReferencesTest() 
{}

bool MovementReferencesTest::setup(yarp::os::Property &config)
{

    // initialization goes here ...


    if(config.check("name"))
        setName(config.find("name").asString());
    else
        setName("MovementReferencesTest");
        
        
    // updating parameters
    RTF_ASSERT_ERROR_IF(config.check("robot"),  "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(config.check("part"),   "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(config.check("joints"), "The joints list must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(config.check("home"),   "The home position must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF(config.check("target"), "Missing 'target' parameter, cannot open device");
    
    robotName = config.find("robot").asString();
    partName  = config.find("part").asString();
    numJoints = config.find("joints").asInt();

    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/moveRefTest/"+robotName+"/"+partName);

    dd = new yarp::dev::PolyDriver(options);
    RTF_ASSERT_ERROR_IF(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF(dd->view(iPosition2),"Unable to open openloop interface");
    RTF_ASSERT_ERROR_IF(dd->view(iEncoders),"Unable to open encoders interface");
    RTF_ASSERT_ERROR_IF(dd->view(iControlMode2),"Unable to open control mode interface");
    RTF_ASSERT_ERROR_IF(dd->view(iOpenLoop),"Unable to open OpnLoop interface");
    RTF_ASSERT_ERROR_IF(dd->view(iPosDirect),"Unable to open OpnLoop interface");
    RTF_ASSERT_ERROR_IF(dd->view(iVelocity2),"Unable to open velocity2 interface");
    


    if (!iEncoders->getAxes(&numJointsInPart))
    {
        RTF_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    Bottle* homeBottle = config.find("home").asList();
    RTF_ASSERT_ERROR_IF(homeBottle!=0,"unable to parse home parameter");

    Bottle* jointsBottle = config.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* targetBottle = config.find("target").asList();
    RTF_ASSERT_ERROR_IF(targetBottle!=0,"unable to parse target parameter");
    
    Bottle* refAccBottle = config.find("refAcc").asList();
    RTF_ASSERT_ERROR_IF(refAccBottle!=0,"unable to parse refAcc parameter");
    
    Bottle* refVelBottle = config.find("refVel").asList();
    RTF_ASSERT_ERROR_IF(refVelBottle!=0,"unable to parse refVel parameter");
    

    int numJoints = jointsBottle->size();
    RTF_ASSERT_ERROR_IF(numJoints>0 && numJoints<=numJointsInPart,"invalid number of joints, it must be >0 & <= number of part joints");
    jointsList.clear();
    for (int i=0; i <numJoints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    homePos.resize (numJoints);         for (int i=0; i< numJoints; i++) homePos[i]=homeBottle->get(i).asDouble();
    targetPos.resize (numJoints);       for (int i=0; i< numJoints; i++) targetPos[i]=targetBottle->get(i).asDouble();
    refVel.resize (numJoints);          for (int i=0; i< numJoints; i++) refVel[i]=refVelBottle->get(i).asDouble();
    refAcc.resize (numJoints);          for (int i=0; i< numJoints; i++) refAcc[i]=refAccBottle->get(i).asDouble();
    
    jPosMotion = new jointsPosMotion(dd, jointsList);
    jPosMotion->setTolerance(2.0);
    jPosMotion->setTimeout(5); //5 sec


    return true;
}

void MovementReferencesTest::tearDown() {

    // finalization goes her ...
    if(jPosMotion)
        delete jPosMotion;
}

void MovementReferencesTest::run() {

    int nJoints=0;
    bool doneAll=false;
    bool ret=false;

    RTF_TEST_REPORT("setting velocity and acceleration refs for all joints...");
    
    RTF_TEST_FAIL_IF(iVelocity2->setRefAccelerations(numJoints, (int*)jointsList.data(), refAcc.data()),
                Asserter::format("setting reference acceleration on joints"));
    
    RTF_TEST_FAIL_IF(iPosition2->setRefSpeeds(numJoints, (int*)jointsList.data(), refVel.data()),
                Asserter::format("setting reference speed on joints"));
    
    

    RTF_TEST_REPORT("Checking individual joints...");
    
    for (int i=0; i<numJoints; ++i)
    {
        double reached_pos;
        double rec_targetPos;
        
        int rec_mode;
        
    // 1) check get reference position returns the target position set by positionMove(..)
        RTF_TEST_REPORT(Asserter::format("Checking PosReference joint %d", jointsList[i]));
        //Note: gotosingle use IPositioncontrol2::PositionMove
        RTF_TEST_FAIL_IF(jPosMotion->goToSingle(jointsList[i], targetPos[i], &reached_pos),
                Asserter::format(("go to target pos  for j %d"),jointsList[i])); 
        RTF_TEST_FAIL_IF(iPosition2->getTargetPosition(jointsList[i], &rec_targetPos),
                Asserter::format(("getting target pos for j %d"),jointsList[i])); 
        
        RTF_TEST_FAIL_IF((targetPos[i] == rec_targetPos),
                Asserter::format(("IPositionControl2: getting target pos for j %d: setval =%.2f received %.2f"),jointsList[i], targetPos[i],rec_targetPos));
        
        RTF_TEST_REPORT(Asserter::format("Checking openloop reference joint %d", jointsList[i]));
    //2) check get reference output (openloop mode) returns the ouput set by setRefOutput
        //set opnLoop mode        
        RTF_TEST_FAIL_IF(iControlMode2->setControlMode(jointsList[i], VOCAB_CM_OPENLOOP),
                Asserter::format(("setting control mode for j %d"),jointsList[i])); 

        RTF_TEST_FAIL_IF(iControlMode2->getControlMode(jointsList[i], &rec_mode),
               Asserter::format(("getting control mode for j %d"),jointsList[i])); 
                
        RTF_ASSERT_FAIL_IF((rec_mode == VOCAB_CM_OPENLOOP),
               Asserter::format(("joint %d: is not in openloop"),jointsList[i]));
        
        double output = 5;
        double rec_output;
        RTF_TEST_FAIL_IF(iOpenLoop->setRefOutput(jointsList[i], output),
               Asserter::format(("set ref output for j %d"),jointsList[i])); 
        RTF_TEST_FAIL_IF(iOpenLoop->getRefOutput(jointsList[i], &rec_output),
               Asserter::format(("get ref output for j %d"),jointsList[i])); 
        
        RTF_TEST_FAIL_IF((output == rec_output),
               Asserter::format(("getting target output for j %d: setval =%.2f received %.2f"),jointsList[i], output,rec_output));

        RTF_TEST_FAIL_IF(iPosition2->positionMove(jointsList[i], homePos[i]),
       Asserter::format(("go to home  for j %d"),jointsList[i])); 
        RTF_TEST_FAIL_IF(iPosition2->getTargetPosition(jointsList[i], &rec_targetPos),
               Asserter::format(("getting target pos for j %d"),jointsList[i]));
        
        //here I expect getTargetPosition returns targetPos[j] and not homePos[j] because joint is in openLoop control mode and 
        //the positionMove(homepos) command should be discarded by firmware motor controller
        RTF_TEST_FAIL_IF((targetPos[i] == rec_targetPos),
               Asserter::format(("joint %d discards PosotinMove command while it is in opnLoop mode"),jointsList[i]));

    //3) check get reference pos (directPosition mode) returns the target position set by setPosition()
        //set direct mode
        RTF_TEST_REPORT(Asserter::format("Checking directPos reference joint %d", jointsList[i]));        
        RTF_TEST_FAIL_IF(iControlMode2->setControlMode(jointsList[i], VOCAB_CM_POSITION_DIRECT),
               Asserter::format(("setting control mode for j %d"),jointsList[i])); 
        RTF_TEST_FAIL_IF(iControlMode2->getControlMode(jointsList[i], &rec_mode),
               Asserter::format(("getting control mode for j %d"),jointsList[i])); 
                
        RTF_ASSERT_FAIL_IF((rec_mode == VOCAB_CM_POSITION_DIRECT),
               Asserter::format(("joint %d: is not in direct mode"),jointsList[i]));
        
        double curr_pos;
        RTF_TEST_FAIL_IF(iEncoders->getEncoder(jointsList[i], &curr_pos),
               Asserter::format(("get encoders for j %d"),jointsList[i])); 
                
        double delta = 0.1;
        double new_directPos = curr_pos+delta;
        RTF_TEST_FAIL_IF(iPosDirect->setPosition(jointsList[i], new_directPos),
               Asserter::format(("Direct:setPosition for j %d"),jointsList[i])); 
        RTF_TEST_FAIL_IF(iPosDirect->getRefPosition(jointsList[i], &rec_targetPos),
               Asserter::format(("getting target pos for j %d"),jointsList[i])); 
        
        RTF_TEST_FAIL_IF((new_directPos == rec_targetPos),
               Asserter::format(("iDirect: getting target direct pos for j %d: setval =%.2f received %.2f"),jointsList[i], new_directPos,rec_targetPos));

        //here I'm going to check the position reference is not changed.
        RTF_TEST_FAIL_IF(iPosition2->getTargetPosition(jointsList[i], &rec_targetPos),
               Asserter::format(("getting target pos for j %d"),jointsList[i])); 
        RTF_TEST_FAIL_IF((targetPos[i] == rec_targetPos),
               Asserter::format(("joint %d discards PosotinMove command while it is in opnLoop mode"),jointsList[i]));
                
    //4) check get reference velocity (velocity mode) returns the target velocity set by velocityMove()
        //set velocity mode
        RTF_TEST_REPORT(Asserter::format("Checking velocity reference joint %d", jointsList[i]));        
        RTF_TEST_FAIL_IF(iControlMode2->setControlMode(jointsList[i], VOCAB_CM_VELOCITY),
               Asserter::format(("setting control mode for j %d"),jointsList[i])); 
        RTF_TEST_FAIL_IF(iControlMode2->getControlMode(jointsList[i], &rec_mode),
               Asserter::format(("getting control mode for j %d"),jointsList[i])); 
                
        RTF_ASSERT_FAIL_IF((rec_mode == VOCAB_CM_VELOCITY),
               Asserter::format(("joint %d: is not in velocity mode"),jointsList[i]));

        double vel= 0.5;
        double rec_vel;
        RTF_TEST_FAIL_IF(iVelocity2->velocityMove(jointsList[i], vel),
               Asserter::format(("IVelocity:velocityMove for j %d"),jointsList[i])); 
        RTF_TEST_FAIL_IF(iVelocity2->getRefVelocity(jointsList[i], &rec_vel),
               Asserter::format(("IVelocity:getting target velocity for j %d"),jointsList[i]));
        RTF_TEST_FAIL_IF((vel == rec_vel),
               Asserter::format(("iVelocity: getting target vel for j %d: setval =%.2f received %.2f"),jointsList[i], vel[i],rec_vel));     
    }
}
