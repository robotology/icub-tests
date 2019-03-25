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

#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include "iKiniDynConsistencyTest.h"

// Yarp includes
#include <yarp/math/api.h>
#include <yarp/math/Math.h>
#include <yarp/os/Random.h>
#include <yarp/os/Time.h>

// iKin includes
#include <iCub/iKin/iKinFwd.h>

// iDyn includes
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::iDyn;


// some utils
void set_random_vector(yarp::sig::Vector & vec, yarp::os::Random & rng, double coeff=0.0)
{
    for( int i=0; i < (int)vec.size(); i++ ) {
        vec[i] =  coeff*M_PI*rng.uniform();
    }
}


// prepare the plugin
PREPARE_PLUGIN(iKiniDynConsistencyTest)

iKiniDynConsistencyTest::iKiniDynConsistencyTest() : TestCase("iKiniDynConsistencyTest") {
}

void iKiniDynConsistencyTest::check_matrix_are_equal(const yarp::sig::Matrix & mat1,
                            const yarp::sig::Matrix & mat2,
                            double tol)
{
    RTF_TEST_REPORT(Asserter::format("Comparing mat1: \n %s \n",mat1.toString().c_str()));
    RTF_TEST_REPORT(Asserter::format("with mat2: \n %s \n",mat2.toString().c_str()));
    RTF_TEST_FAIL_IF_FALSE(mat1.rows()==mat2.rows(),"matrix rows do not match");
    RTF_TEST_FAIL_IF_FALSE(mat1.cols()==mat2.cols(),"matrix cols do not match");
    for(int row=0; row < mat1.rows(); row++ )
    {
        for(int col=0; col < mat1.cols(); col++ )
        {
            RTF_TEST_FAIL_IF_FALSE(std::fabs(mat1(row,col)-mat2(row,col)) < tol,
                      Asserter::format("Element %d %d don't match",row,col));
        }
    }

}


iKiniDynConsistencyTest::~iKiniDynConsistencyTest() { }

bool iKiniDynConsistencyTest::setup(int argc, char** argv) {

    return true;
}

void iKiniDynConsistencyTest::tearDown() {
    // finalization goes her ...
}


Matrix iKiniDynConsistencyTest::getiKinTransform(const string part, int index)
{
    if( part == "left_leg" )
    {
        return ikin_lleg.getH(index);
    }
    if( part == "right_leg" )
    {
        return ikin_rleg.getH(index);
    }
    if( part == "left_arm" )
    {
        return ikin_larm.getH(index);
    }
    if( part == "right_arm" )
    {
        return ikin_rarm.getH(index);
    }
    return Matrix();
}

Matrix iKiniDynConsistencyTest::getiDynTransform(const string part, int index)
{
    if( part == "left_leg" )
    {
        return icub->lowerTorso->HLeft*
               icub->lowerTorso->left->getH();
    }
    if( part == "right_leg" )
    {
        return icub->lowerTorso->HRight*
               icub->lowerTorso->right->getH();
    }
    if( part == "left_arm" )
    {
        //return ikin_larm.getH(index);
    }
    if( part == "right_arm" )
    {
        //return ikin_rarm.getH(index);
    }
    return Matrix();
}

void iKiniDynConsistencyTest::run() {

    RTF_TEST_REPORT("Creating iCubWholeBody object");
    version_tag ver;
    ver.head_version = 1;
    ver.legs_version = 1;
    RTF_TEST_REPORT(Asserter::format("Creating iCubWholeBody object with head version %d and legs version %d", ver.head_version, ver.legs_version));
    icub = new iCubWholeBody(ver);

    // now we set the joint angles for all the limbs
    // if connected to the real robot, we can take this values from the encoders
    q_head.resize(icub->upperTorso->getNLinks("head"));
    q_larm.resize(icub->upperTorso->getNLinks("left_arm"));
    q_rarm.resize(icub->upperTorso->getNLinks("right_arm"));
    q_torso.resize(icub->lowerTorso->getNLinks("torso"));
    q_lleg.resize(icub->lowerTorso->getNLinks("left_leg"));
    q_rleg.resize(icub->lowerTorso->getNLinks("right_leg"));

    //We can initialize the joint position to random values

    yarp::os::Random rng;
    rng.seed(147);
    double coeff = 1.0;
    set_random_vector(q_head,rng,coeff);
    set_random_vector(q_larm,rng,coeff);
    set_random_vector(q_rarm,rng,coeff);
    set_random_vector(q_torso,rng,coeff);
    set_random_vector(q_lleg,rng,coeff);
    set_random_vector(q_rleg,rng,coeff);



    RTF_TEST_REPORT("Setting positions in iCubWholeBody");
    q_head = icub->upperTorso->setAng("head",q_head);
    q_rarm = icub->upperTorso->setAng("right_arm",q_rarm);
    q_larm = icub->upperTorso->setAng("left_arm",q_larm);
    q_torso = icub->lowerTorso->setAng("torso",q_torso);
    q_rleg = icub->lowerTorso->setAng("right_leg",q_rleg);
    q_lleg = icub->lowerTorso->setAng("left_leg",q_lleg);

    yarp::sig::Matrix transform_ikin(4,4), transform_idyn(4,4);

    //////////////////////////////////////////////////////////////////////////
    RTF_TEST_REPORT("Checking left hand position");
    new(&ikin_larm) iCubArm("left");

    Vector q_torso_larm = cat(q_torso,q_larm);
    ikin_larm.setAllConstraints(false);
    ikin_larm.releaseLink(0);
    ikin_larm.releaseLink(1);
    ikin_larm.releaseLink(2);
    RTF_TEST_REPORT(Asserter::format("q_torso_larm : %d ikin_larm : %d",q_torso_larm.size(),ikin_larm.getDOF()));
    RTF_ASSERT_ERROR_IF_FALSE(q_torso_larm.size() == ikin_larm.getDOF(),"unexpected chain size");
    ikin_larm.setAng(q_torso_larm);

    transform_ikin = ikin_larm.getH();
    transform_idyn = icub->lowerTorso->HUp*
                     icub->lowerTorso->up->getH(2,true)*
                     icub->upperTorso->HLeft*
                     icub->upperTorso->left->getH();

    check_matrix_are_equal(transform_ikin,transform_idyn);


    //////////////////////////////////////////////////////////////////////////
    RTF_TEST_REPORT("Checking right hand position");
    new(&ikin_rarm) iCubArm("right");
    Vector q_torso_rarm = cat(q_torso,q_rarm);
    ikin_rarm.setAllConstraints(false);
    ikin_rarm.releaseLink(0);
    ikin_rarm.releaseLink(1);
    ikin_rarm.releaseLink(2);
    RTF_ASSERT_ERROR_IF_FALSE(q_torso_rarm.size() == ikin_rarm.getDOF(),"unexpected chain size");
    ikin_rarm.setAng(q_torso_rarm);

    transform_ikin = ikin_rarm.getH();
    transform_idyn = icub->lowerTorso->HUp*
                     icub->lowerTorso->up->getH(2,true)*
                     icub->upperTorso->HRight*
                     icub->upperTorso->right->getH();

    check_matrix_are_equal(transform_ikin,transform_idyn);


    //////////////////////////////////////////////////////////////////////////
    RTF_TEST_REPORT("Checking left leg end effector positions");
    new(&ikin_lleg) iCubLeg("left");
    ikin_lleg.setAllConstraints(false);
    RTF_TEST_REPORT(Asserter::format("q_lleg : %d ikin_lleg : %d",q_lleg.size(),ikin_lleg.getDOF()));
    RTF_ASSERT_ERROR_IF_FALSE(q_lleg.size() == ikin_lleg.getDOF(),"unexpected chain size");
    ikin_lleg.setAng(q_lleg);

    transform_ikin = ikin_lleg.getH();
    transform_idyn = icub->lowerTorso->HLeft*
                     icub->lowerTorso->left->getH();

    check_matrix_are_equal(transform_ikin,transform_idyn);

    /*
    for(int link=0; link < ikin_lleg.getDOF(); link++ )
    {
        RTF_TEST_REPORT(Asserter::format("Checking %d DH frame of left_leg",link));
        check_matrix_are_equal(this->getiKinTransform("left_leg",link),
                               this->getiDynTransform("left_leg",link));
    }*/

    //////////////////////////////////////////////////////////////////////////
    RTF_TEST_REPORT("Checking right leg end effector positions");
    new(&ikin_rleg) iCubLeg("right");
    ikin_rleg.setAllConstraints(false);
    RTF_ASSERT_ERROR_IF_FALSE(q_rleg.size() == ikin_rleg.getDOF(),"unexpected chain size");
    ikin_rleg.setAng(q_rleg);

    transform_ikin = ikin_rleg.getH();
    transform_idyn = icub->lowerTorso->HRight*
                     icub->lowerTorso->right->getH();

    check_matrix_are_equal(transform_ikin,transform_idyn);

    /*
    for(int link=0; link < ikin_rleg.getDOF(); link++ )
    {
        RTF_TEST_REPORT(Asserter::format("Checking %d DH frame of right_leg",link));
        check_matrix_are_equal(this->getiKinTransform("right_leg",link),
                               this->getiDynTransform("right_leg",link));
    }*/

    delete icub;
}

