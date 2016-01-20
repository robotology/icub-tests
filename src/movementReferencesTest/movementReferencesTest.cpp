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


}

MovementReferencesTest::~MovementReferencesTest() 
{
}

bool MovementReferencesTest::setup(yarp::os::Property &config)
{

    // initialization goes here ...
    numJointsInPart = 0;
    jPosMotion=NULL;

    iEncoders=NULL;
    iPosition2=NULL;
    iPosDirect=NULL;
    iControlMode2=NULL;
    iVelocity2=NULL;
    initialized=false;

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
    if(homeBottle == NULL)
    {

        RTF_TEST_REPORT("home bottle is null");
    }
    else
    {

        RTF_TEST_REPORT("home bottle is not null");
    }

    RTF_ASSERT_ERROR_IF(homeBottle!=0,"unable to parse home parameter");

    Bottle* jointsBottle = config.find("joints").asList();
    RTF_ASSERT_ERROR_IF(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* targetBottle = config.find("target").asList();
    RTF_ASSERT_ERROR_IF(targetBottle!=0,"unable to parse target parameter");
    
    Bottle* refAccBottle = config.find("refAcc").asList();
    if(refAccBottle == NULL)
        RTF_TEST_REPORT("refAcc param not present. Test will use deafoult values.");
    
    Bottle* refVelBottle = config.find("refVel").asList();
    RTF_ASSERT_ERROR_IF(refVelBottle!=0,"unable to parse refVel parameter");
    

    numJoints = jointsBottle->size();
    RTF_TEST_REPORT(Asserter::format("num joints: %d", numJoints));


    RTF_ASSERT_ERROR_IF(numJoints>0 && numJoints<=numJointsInPart,"invalid number of joints, it must be >0 & <= number of part joints");

    //jointsList.clear();
    targetPos.clear();

    jointsList.resize(numJoints);
    for (int i=0; i< numJoints; i++)
    {
        jointsList[i]=jointsBottle->get(i).asInt();
    }

    jList = new int[numJoints];
    for (int i=0; i< numJoints; i++)
    {
        jList[i]=jointsBottle->get(i).asInt();
    }



    //for (int i=0; i <numJoints; i++) jointsList.push_back(jointsBottle->get(i).asInt());

    homePos.resize (numJoints);         for (int i=0; i< numJoints; i++) homePos[i]=homeBottle->get(i).asDouble();
    targetPos.resize (numJoints);       for (int i=0; i< numJoints; i++) targetPos[i]=targetBottle->get(i).asDouble();
    refVel.resize (numJoints);          for (int i=0; i< numJoints; i++) refVel[i]=refVelBottle->get(i).asDouble();
    if(refAccBottle != NULL)
    {
        refAcc.resize (numJoints);
        for (int i=0; i< numJoints; i++) refAcc[i]=refAccBottle->get(i).asDouble();
    }
    else
    {
        refAcc.resize (0); 
    }
    
    jPosMotion = new jointsPosMotion(dd, jointsList);
    jPosMotion->setTolerance(2.0);
    jPosMotion->setTimeout(5); //5 sec

//    RTF_TEST_REPORT(jointsList.toString().c_str());
//    RTF_TEST_REPORT(Asserter::format("startup num joints: %d partnj=%d", numJoints, numJointsInPart));
//    RTF_TEST_REPORT(targetPos.toString().c_str());
//    RTF_TEST_REPORT(refVel.toString().c_str());
//    RTF_TEST_REPORT(refAcc.toString().c_str());

//    for(int k=0; k<numJoints; k++)
//    {
//        RTF_TEST_REPORT(Asserter::format("targetpos [%d]=%.2f", k, targetPos[k]));
//    }

 /*   for(int k=0; k<numJoints; k++)
    {
        RTF_TEST_REPORT(Asserter::format("jointlist [%d]=%d", k, (int)jointsList[k]));
        RTF_TEST_REPORT(Asserter::format("jlist [%d]=%d", k, jList[k]));
    }*/
    return true;
}

void MovementReferencesTest::tearDown() {

    // finalization goes her ...

    for (int i=0; i<numJoints; ++i)
    {

    // 1) check get reference position returns the target position set by positionMove(..)

        RTF_TEST_REPORT(Asserter::format("tear down:  joint %d is going to home", jList[i]));

        setAndCheckControlMode(jList[i], VOCAB_CM_POSITION);

        RTF_TEST_FAIL_IF(jPosMotion->goToSingle(jList[i], homePos[i]),
                Asserter::format(("go to target pos  for j %d"),jList[i]));
    }


    if(jPosMotion)
        delete jPosMotion;
}


void MovementReferencesTest::setAndCheckControlMode(int j, int mode)
{
    int rec_mode=VOCAB_CM_IDLE;
    RTF_TEST_FAIL_IF(iControlMode2->setControlMode(j, mode),
            Asserter::format(("setting control mode for j %d"),j));

    yarp::os::Time::delay(0.1);

    RTF_TEST_FAIL_IF(iControlMode2->getControlMode(j, &rec_mode),
           Asserter::format(("getting control mode for j %d"),j));

    RTF_ASSERT_FAIL_IF((rec_mode == mode),
           Asserter::format(("joint %d: is not in position"),j));

}

void MovementReferencesTest::run() {

    int nJoints=0;
    bool doneAll=false;
    bool ret=false;

//    RTF_TEST_REPORT(jointsList.toString().c_str());
//    for(int k=0; k<numJoints; k++)
//    {
//        RTF_TEST_REPORT(Asserter::format("targetpos [%d]=%.2f", k, targetPos[k]));
//    }

//    for(int k=0; k<numJoints; k++)
//    {
//        RTF_TEST_REPORT(Asserter::format("jointlist [%d]=%d", k, jointsList[k]));
//    }


    RTF_TEST_REPORT("setting velocity refs for all joints...");
    
    if(refAcc.size() != 0)
    {
        RTF_TEST_REPORT("setting acceleration refs for all joints...");
        RTF_TEST_FAIL_IF(iVelocity2->setRefAccelerations(numJoints, jList, refAcc.data()),
                Asserter::format("setting reference acceleration on joints"));
    }

    RTF_TEST_REPORT("setting velocity refs for all joints...");
    RTF_TEST_FAIL_IF(iPosition2->setRefSpeeds(numJoints, jList, refVel.data()),
                Asserter::format("setting reference speed on joints"));
    



    RTF_TEST_REPORT("all joints are going to home...");
    for (int i=0; i<numJoints; ++i)
    {

    // 1) check get reference position returns the target position set by positionMove(..)

        RTF_TEST_REPORT(Asserter::format(" joint %d is going to home", jList[i]));

        setAndCheckControlMode(jList[i], VOCAB_CM_POSITION);

        RTF_ASSERT_FAIL_IF(jPosMotion->goToSingle(jList[i], homePos[i]),
                Asserter::format(("go to target pos  for j %d"),jList[i]));
    }
    
    yarp::os::Time::delay(5);

    RTF_TEST_REPORT("Checking individual joints...");

    const double res_th = 0.01; //resolution threshold
    //numJoints=numJointsInPart;
    for (int i=0; i<numJoints; ++i)
    {
        //double reached_pos;
        double rec_targetPos=200.0;

    // 1) check get reference position returns the target position set by positionMove(..)

        RTF_TEST_REPORT(Asserter::format("Checking PosReference joint %d with resolution threshold %.3f", jList[i], res_th));

        setAndCheckControlMode(jList[i], VOCAB_CM_POSITION);

        RTF_TEST_FAIL_IF(jPosMotion->goToSingle(jList[i], targetPos[i]),
                Asserter::format(("go to target pos  for j %d"),jList[i])); //Note: gotosingle use IPositioncontrol2::PositionMove
        
    yarp::os::Time::delay(0.5);

        RTF_TEST_FAIL_IF(iPosition2->getTargetPosition(jList[i], &rec_targetPos),
                Asserter::format(("getting target pos for j %d"),jList[i]));
        
        bool res = YarpTestAsserter::isApproxEqual(targetPos[i], rec_targetPos, res_th, res_th);
        RTF_TEST_CHECK(res, Asserter::format(
                           ("IPositionControl2: getting target pos for j %d: setval =%.2f received %.2f"),
                           jList[i], targetPos[i],rec_targetPos));
        
        //if(!res)
        //    std::cout <<"ERRORE: getTargetPosition: j " << jList[i] << "sent" << targetPos[i] << "rec" << rec_targetPos;
        //else
        //    std::cout <<"OK: getTargetPosition: j " << jList[i] << "sent" << targetPos[i] << "rec" << rec_targetPos;

        yarp::os::Time::delay(3);

    //2) check get reference output (openloop mode) returns the ouput set by setRefOutput
        RTF_TEST_REPORT(Asserter::format("Checking openloop reference joint %d", jList[i]));


        setAndCheckControlMode(jList[i], VOCAB_CM_OPENLOOP);
        
        double output = 5;
        double rec_output = 0;
        RTF_TEST_FAIL_IF(iOpenLoop->setRefOutput(jList[i], output),
               Asserter::format(("set ref output for j %d"),jList[i]));
yarp::os::Time::delay(0.5);

        RTF_TEST_FAIL_IF(iOpenLoop->getRefOutput(jList[i], &rec_output),
               Asserter::format(("get ref output for j %d"),jList[i]));
        
        RTF_TEST_CHECK((output == rec_output),
               Asserter::format(("getting target output for j %d: setval =%.2f received %.2f"),jList[i], output,rec_output));

        RTF_TEST_FAIL_IF(iPosition2->positionMove(jList[i], homePos[i]),
                Asserter::format(("go to home  for j %d"),jList[i]));
yarp::os::Time::delay(0.5);
        RTF_TEST_FAIL_IF(iPosition2->getTargetPosition(jList[i], &rec_targetPos),
               Asserter::format(("getting target pos for j %d"),jList[i]));
        
        //here I expect getTargetPosition returns targetPos[j] and not homePos[j] because joint is in openLoop control mode and 
        //the positionMove(homepos) command should be discarded by firmware motor controller
        res = YarpTestAsserter::isApproxEqual(homePos[i], rec_targetPos, res_th, res_th);
        RTF_TEST_CHECK(!res,
               Asserter::format(("joint %d discards PosotinMove command while it is in opnLoop mode"),jList[i]));

    //3) check get reference pos (directPosition mode) returns the target position set by setPosition()
        //set direct mode
        RTF_TEST_REPORT(Asserter::format("Checking directPos reference joint %d", jList[i]));

        setAndCheckControlMode(jList[i], VOCAB_CM_POSITION_DIRECT);
        
        double curr_pos=targetPos[i];
        RTF_TEST_FAIL_IF(iEncoders->getEncoder(jList[i], &curr_pos),
               Asserter::format(("get encoders for j %d"),jList[i]));
                
        double delta = 0.1;
        double new_directPos = curr_pos+delta;
        RTF_TEST_FAIL_IF(iPosDirect->setPosition(jList[i], new_directPos),
               Asserter::format(("Direct:setPosition for j %d"),jList[i]));
yarp::os::Time::delay(0.5);

        RTF_TEST_FAIL_IF(iPosDirect->getRefPosition(jList[i], &rec_targetPos),
               Asserter::format(("getting target pos for j %d"),jList[i]));
        
        res = YarpTestAsserter::isApproxEqual(new_directPos, rec_targetPos, res_th, res_th);
        RTF_TEST_CHECK(res,
               Asserter::format(("iDirect: getting target direct pos for j %d: setval =%.2f received %.2f"),jList[i], new_directPos,rec_targetPos));

        //here I'm going to check the position reference is not changed.
        RTF_TEST_FAIL_IF(iPosition2->getTargetPosition(jList[i], &rec_targetPos),
               Asserter::format(("getting target pos for j %d"),jList[i]));

        res = YarpTestAsserter::isApproxEqual(targetPos[i], rec_targetPos, res_th, res_th);
        RTF_TEST_CHECK(res, Asserter::format(
                           ("IPositionControl2: getting target pos for j %d: setval =%.2f received %.2f"),
                           jList[i], targetPos[i],rec_targetPos));

    //4) check get reference velocity (velocity mode) returns the target velocity set by velocityMove()
        //set velocity mode
        RTF_TEST_REPORT(Asserter::format("Checking velocity reference joint %d", jList[i]));

        setAndCheckControlMode(jList[i], VOCAB_CM_VELOCITY);

        double vel= 0.5;
        double rec_vel;
        RTF_TEST_FAIL_IF(iVelocity2->velocityMove(jList[i], vel),
               Asserter::format(("IVelocity:velocityMove for j %d"),jList[i]));

yarp::os::Time::delay(0.5);
        RTF_TEST_FAIL_IF(iVelocity2->getRefVelocity(jList[i], &rec_vel),
               Asserter::format(("IVelocity:getting target velocity for j %d"),jList[i]));
        res = YarpTestAsserter::isApproxEqual(vel, rec_vel, res_th, res_th);
        RTF_TEST_CHECK(res,
               Asserter::format(("iVelocity: getting target vel for j %d: setval =%.2f received %.2f"),jList[i], vel,rec_vel));
    }

}
