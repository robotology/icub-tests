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

#include <cmath>

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>
#include <yarp/robottestingframework/TestAsserter.h>

#include "movementReferencesTest.h"

using namespace std;
using namespace robottestingframework;

using namespace yarp::os;


// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(MovementReferencesTest)

MovementReferencesTest::MovementReferencesTest() : yarp::robottestingframework::TestCase("MovementReferencesTest")
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
    iPosition=NULL;
    iPosDirect=NULL;
    iControlMode=NULL;
    iVelocity=NULL;
    initialized=false;

    if(config.check("name"))
        setName(config.find("name").asString());
    else
        setName("MovementReferencesTest");


    // updating parameters
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(config.check("robot"),  "The robot name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(config.check("part"),   "The part name must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(config.check("joints"), "The joints list must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(config.check("home"),   "The home position must be given as the test parameter!");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(config.check("target"), "Missing 'target' parameter, cannot open device");

    robotName = config.find("robot").asString();
    partName  = config.find("part").asString();


    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/moveRefTest/"+robotName+"/"+partName);

    dd = new yarp::dev::PolyDriver(options);
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->isValid(),"Unable to open device driver");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iPosition),"Unable to open position interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iEncoders),"Unable to open encoders interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iControlMode),"Unable to open control mode interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iPWM), "Unable to open PWM interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iPosDirect),"Unable to open OpnLoop interface");
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(dd->view(iVelocity),"Unable to open velocity2 interface");



    if (!iEncoders->getAxes(&numJointsInPart))
    {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("unable to get the number of joints of the part");
    }

    Bottle* homeBottle = config.find("home").asList();
    if(homeBottle == NULL)
    {

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("home bottle is null");
    }
    else
    {

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("home bottle is not null");
    }

    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(homeBottle!=0,"unable to parse home parameter");

    Bottle* jointsBottle = config.find("joints").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(jointsBottle!=0,"unable to parse joints parameter");

    Bottle* targetBottle = config.find("target").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(targetBottle!=0,"unable to parse target parameter");

    Bottle* refAccBottle = config.find("refAcc").asList();
    if(refAccBottle == NULL)
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("refAcc param not present. Test will use deafoult values.");

    Bottle* refVelBottle = config.find("refVel").asList();
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(refVelBottle!=0,"unable to parse refVel parameter");


    numJoints = jointsBottle->size();
    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("num joints: %d", numJoints));


    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(numJoints>0 && numJoints<=numJointsInPart,"invalid number of joints, it must be >0 & <= number of part joints");

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

    jPosMotion = new yarp::robottestingframework::jointsPosMotion(dd, jointsList);
    jPosMotion->setTolerance(2.0);
    jPosMotion->setTimeout(5); //5 sec

//    ROBOTTESTINGFRAMEWORK_TEST_REPORT(jointsList.toString().c_str());
//    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("startup num joints: %d partnj=%d", numJoints, numJointsInPart));
//    ROBOTTESTINGFRAMEWORK_TEST_REPORT(targetPos.toString().c_str());
//    ROBOTTESTINGFRAMEWORK_TEST_REPORT(refVel.toString().c_str());
//    ROBOTTESTINGFRAMEWORK_TEST_REPORT(refAcc.toString().c_str());

//    for(int k=0; k<numJoints; k++)
//    {
//        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("targetpos [%d]=%.2f", k, targetPos[k]));
//    }

 /*   for(int k=0; k<numJoints; k++)
    {
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("jointlist [%d]=%d", k, (int)jointsList[k]));
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("jlist [%d]=%d", k, jList[k]));
    }*/
    return true;
}

void MovementReferencesTest::tearDown() {

    // finalization goes her ...

    for (int i=0; i<numJoints; ++i)
    {

    // 1) check get reference position returns the target position set by positionMove(..)

        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("tear down:  joint %d is going to home", jList[i]));

        setAndCheckControlMode(jList[i], VOCAB_CM_POSITION);

        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(jPosMotion->goToSingle(jList[i], homePos[i]),
                Asserter::format(("go to target pos  for j %d"),jList[i]));
    }


    if(jPosMotion)
        delete jPosMotion;
}


void MovementReferencesTest::setAndCheckControlMode(int j, int mode)
{
    int rec_mode=VOCAB_CM_IDLE;
    ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iControlMode->setControlMode(j, mode),
            Asserter::format(("setting control mode for j %d"),j));

    yarp::os::Time::delay(0.1);

    ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iControlMode->getControlMode(j, &rec_mode),
           Asserter::format(("getting control mode for j %d"),j));

    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE((rec_mode == mode),
           Asserter::format(("joint %d: is not in position"),j));

}

void MovementReferencesTest::run() {

    int nJoints=0;
    bool doneAll=false;
    bool ret=false;

//    ROBOTTESTINGFRAMEWORK_TEST_REPORT(jointsList.toString().c_str());
//    for(int k=0; k<numJoints; k++)
//    {
//        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("targetpos [%d]=%.2f", k, targetPos[k]));
//    }

//    for(int k=0; k<numJoints; k++)
//    {
//        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("jointlist [%d]=%d", k, jointsList[k]));
//    }


    ROBOTTESTINGFRAMEWORK_TEST_REPORT("setting velocity refs for all joints...");

    if(refAcc.size() != 0)
    {
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("setting acceleration refs for all joints...");
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iVelocity->setRefAccelerations(numJoints, jList, refAcc.data()),
                Asserter::format("setting reference acceleration on joints"));
    }

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("setting velocity refs for all joints...");
    ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iPosition->setRefSpeeds(numJoints, jList, refVel.data()),
                Asserter::format("setting reference speed on joints"));




    ROBOTTESTINGFRAMEWORK_TEST_REPORT("all joints are going to home...");
    for (int i=0; i<numJoints; ++i)
    {

    // 1) check get reference position returns the target position set by positionMove(..)

        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format(" joint %d is going to home", jList[i]));

        setAndCheckControlMode(jList[i], VOCAB_CM_POSITION);

        ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(jPosMotion->goToSingle(jList[i], homePos[i]),
                Asserter::format(("go to target pos  for j %d"),jList[i]));
    }

    yarp::os::Time::delay(5);

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Checking individual joints...");

    const double res_th = 0.01; //resolution threshold
    //numJoints=numJointsInPart;
    for (int i=0; i<numJoints; ++i)
    {
        //double reached_pos;
        double rec_targetPos=200.0;

    // 1) check get reference position returns the target position set by positionMove(..)

        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Checking PosReference joint %d with resolution threshold %.3f", jList[i], res_th));

        setAndCheckControlMode(jList[i], VOCAB_CM_POSITION);

        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(jPosMotion->goToSingle(jList[i], targetPos[i]),
                Asserter::format(("go to target pos  for j %d"),jList[i])); //Note: gotosingle use IPositioncontrol2::PositionMove

    yarp::os::Time::delay(0.5);

        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iPosition->getTargetPosition(jList[i], &rec_targetPos),
                Asserter::format(("getting target pos for j %d"),jList[i]));

        bool res = yarp::robottestingframework::TestAsserter::isApproxEqual(targetPos[i], rec_targetPos, res_th, res_th);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(res, Asserter::format(
                           ("IPositionControl: getting target pos for j %d: setval =%.2f received %.2f"),
                           jList[i], targetPos[i],rec_targetPos));

        //if(!res)
        //    std::cout <<"ERRORE: getTargetPosition: j " << jList[i] << "sent" << targetPos[i] << "rec" << rec_targetPos;
        //else
        //    std::cout <<"OK: getTargetPosition: j " << jList[i] << "sent" << targetPos[i] << "rec" << rec_targetPos;

        yarp::os::Time::delay(3);

    //2) check get reference output (pwm mode) returns the ouput set by setRefOutput
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Checking pwm reference joint %d", jList[i]));


        setAndCheckControlMode(jList[i], VOCAB_CM_PWM);

        double output = 2;
        double rec_output = 0;
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iPWM->setRefDutyCycle(jList[i], output),
               Asserter::format(("set ref output for j %d"),jList[i]));
yarp::os::Time::delay(0.5);

        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iPWM->getRefDutyCycle(jList[i], &rec_output),
               Asserter::format(("get ref output for j %d"),jList[i]));

        ROBOTTESTINGFRAMEWORK_TEST_CHECK((output == rec_output),
               Asserter::format(("getting target output for j %d: setval =%.2f received %.2f"),jList[i], output,rec_output));

        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iPosition->positionMove(jList[i], homePos[i]),
                Asserter::format(("go to home  for j %d"),jList[i]));
yarp::os::Time::delay(0.5);
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iPosition->getTargetPosition(jList[i], &rec_targetPos),
               Asserter::format(("getting target pos for j %d"),jList[i]));

        //here I expect getTargetPosition returns targetPos[j] and not homePos[j] because joint is in pwm control mode and
        //the positionMove(homepos) command should be discarded by firmware motor controller
        res = yarp::robottestingframework::TestAsserter::isApproxEqual(homePos[i], rec_targetPos, res_th, res_th);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(!res,
               Asserter::format(("joint %d discards PosotinMove command while it is in opnLoop mode. Set=%.2f rec=%.2f"),jList[i], homePos[i], rec_targetPos));

    //3) check get reference pos (directPosition mode) returns the target position set by setPosition()
        //set direct mode
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Checking directPos reference joint %d", jList[i]));

        setAndCheckControlMode(jList[i], VOCAB_CM_POSITION_DIRECT);

        double curr_pos=targetPos[i];
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iEncoders->getEncoder(jList[i], &curr_pos),
               Asserter::format(("get encoders for j %d"),jList[i]));

        double delta = 0.1;
        double new_directPos = curr_pos+delta;
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iPosDirect->setPosition(jList[i], new_directPos),
               Asserter::format(("Direct:setPosition for j %d"),jList[i]));
yarp::os::Time::delay(0.5);

        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iPosDirect->getRefPosition(jList[i], &rec_targetPos),
               Asserter::format(("getting target pos for j %d"),jList[i]));

        res = yarp::robottestingframework::TestAsserter::isApproxEqual(new_directPos, rec_targetPos, res_th, res_th);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(res,
               Asserter::format(("iDirect: getting target direct pos for j %d: setval =%.2f received %.2f"),jList[i], new_directPos,rec_targetPos));

        //here I'm going to check the position reference is not changed.
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iPosition->getTargetPosition(jList[i], &rec_targetPos),
               Asserter::format(("getting target pos for j %d"),jList[i]));

        res = yarp::robottestingframework::TestAsserter::isApproxEqual(targetPos[i], rec_targetPos, res_th, res_th);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(res, Asserter::format(
                           ("IPositionControl: getting target pos for j %d: setval =%.2f received %.2f"),
                           jList[i], targetPos[i],rec_targetPos));

    //4) check get reference velocity (velocity mode) returns the target velocity set by velocityMove()
        //set velocity mode
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Checking velocity reference joint %d", jList[i]));

        setAndCheckControlMode(jList[i], VOCAB_CM_VELOCITY);

        double vel= 0.5;
        double rec_vel;
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iVelocity->velocityMove(jList[i], vel),
               Asserter::format(("IVelocity:velocityMove for j %d"),jList[i]));

yarp::os::Time::delay(0.5);
        ROBOTTESTINGFRAMEWORK_TEST_FAIL_IF_FALSE(iVelocity->getRefVelocity(jList[i], &rec_vel),
               Asserter::format(("IVelocity:getting target velocity for j %d"),jList[i]));
        res = yarp::robottestingframework::TestAsserter::isApproxEqual(vel, rec_vel, res_th, res_th);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(res,
               Asserter::format(("iVelocity: getting target vel for j %d: setval =%.2f received %.2f"),jList[i], vel,rec_vel));
    }

}
