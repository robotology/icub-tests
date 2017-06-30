/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ugo Pattacini <ugo.pattacini@iit.it>
 *          Giulia Vezzani <giulia.vezzani@iit.it>
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <string>
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include "CartesianControlReachingToleranceTest.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

// prepare the plugin
PREPARE_PLUGIN(CartesianControlReachingToleranceTest)


/***********************************************************************************/
CartesianControlReachingToleranceTest::CartesianControlReachingToleranceTest() :
                                       yarp::rtf::TestCase("CartesianControlReachingToleranceTest")
{
}


/***********************************************************************************/
CartesianControlReachingToleranceTest::~CartesianControlReachingToleranceTest()
{
}


/***********************************************************************************/
bool CartesianControlReachingToleranceTest::setup(Property &property)
{
    string robot=property.check("robot",Value("icubSim")).asString();
    string arm=property.check("arm-type",Value("left")).asString();

    Property optCart;
    optCart.put("device","cartesiancontrollerclient");
    optCart.put("remote",("/"+robot+"/"+"cartesianController/"+arm+"_arm"));
    optCart.put("local",("/"+getName()+"/cartesian/"+arm+"_arm"));

    Property optJoint;
    optJoint.put("device","remote_controlboard");
    optJoint.put("remote",("/"+robot+"/"+arm+"_arm"));
    optJoint.put("local",("/"+getName()+"/joint/"+arm+"_arm"));

    RTF_TEST_REPORT(Asserter::format("Opening Cartesian Controller Client for %s_arm",arm.c_str()));
    RTF_ASSERT_ERROR_IF(drvCart.open(optCart),"Unable to open the client!");

    RTF_TEST_REPORT(Asserter::format("Opening Joint Controller Client for %s_arm",arm.c_str()));
    RTF_ASSERT_ERROR_IF(drvJoint.open(optJoint),"Unable to open the client!");
    return true;
}


/***********************************************************************************/
void CartesianControlReachingToleranceTest::tearDown()
{
    RTF_TEST_REPORT("Closing Cartesian Controller Client");
    RTF_ASSERT_FAIL_IF(drvCart.close(),"Unable to close the client!");

    RTF_TEST_REPORT("Closing Joint Controller Client");
    RTF_ASSERT_FAIL_IF(drvJoint.close(),"Unable to close the client!");
}


/***********************************************************************************/
double CartesianControlReachingToleranceTest::compute_error(const Vector &xh, const Vector &oh,
                                                            const Vector &x, const Vector &o)
{
    Matrix H=axis2dcm(o);
    H(0,3)=x[0];
    H(1,3)=x[1];
    H(2,3)=x[2];

    Vector e(6,0.0);
    e[0]=xh[0]-H(0,3);
    e[1]=xh[1]-H(1,3);
    e[2]=xh[2]-H(2,3);
    Matrix Des=axis2dcm(oh);
    Vector ax=dcm2axis(Des*SE3inv(H));
    e[3]=ax[3]*ax[0];
    e[4]=ax[3]*ax[1];
    e[5]=ax[3]*ax[2];

    return norm(e);
}


/***********************************************************************************/
void CartesianControlReachingToleranceTest::run()
{
    ICartesianControl *iarm;
    RTF_TEST_CHECK(drvCart.view(iarm),"Opening the view on the Cartesian device!");

    IEncoders *ienc;
    RTF_TEST_CHECK(drvJoint.view(ienc),"Opening the view on the Joint device!");

    bool done;

    Vector x,o;    
    double t0=Time::now();
    while (Time::now()-t0<5.0)
    {
        done=iarm->getPose(x,o);
        if (done)
            break;
        Time::delay(0.1);
    }
    RTF_TEST_CHECK(done,"Initial pose retrieved!");

    RTF_TEST_REPORT("Setting up the context");
    int context;
    iarm->storeContext(&context);

    Vector dof;
    iarm->getDOF(dof); dof=1.0;
    dof[0]=dof[1]=dof[2]=0.0;
    iarm->setDOF(dof,dof);
    iarm->setTrajTime(1.0);
    iarm->setInTargetTol(0.02);

    double tol;
    iarm->getInTargetTol(&tol);

    Vector pos2reach(3,0.0);
    pos2reach[0]=-0.35;
    pos2reach[1]=0.0;
    pos2reach[2]=0.15;

    Matrix dcm2reach=zeros(3,3);
    dcm2reach(0,0)=dcm2reach(2,1)=dcm2reach(1,2)=-1.0;
    Vector ori2reach=dcm2axis(dcm2reach);

    RTF_TEST_REPORT(Asserter::format("Reaching for the target: (%s, %s)",
                                     pos2reach.toString(3,3).c_str(),ori2reach.toString(3,3).c_str()));
    Vector xh,oh,qh;
    iarm->goToPoseSync(pos2reach,ori2reach);
    iarm->getDesired(xh,oh,qh);

    RTF_TEST_REPORT("Waiting");
    done=iarm->waitMotionDone(0.1,10.0);
    iarm->stopControl();

    Vector xf,of;
    iarm->getPose(xf,of);

    int nJoints;
    ienc->getAxes(&nJoints);
    Vector qf(nJoints);
    ienc->getEncoders(qf.data());

    RTF_TEST_CHECK(done,"Target reached!");
    RTF_TEST_REPORT(Asserter::format("Reaching tolerance: %g",tol));
    RTF_TEST_REPORT(Asserter::format("Solved pose: (%s, %s)",
                                     xh.toString(3,3).c_str(),oh.toString(3,3).c_str()));
    RTF_TEST_REPORT(Asserter::format("Reached pose: (%s, %s)",
                                     xf.toString(3,3).c_str(),of.toString(3,3).c_str()));
    RTF_TEST_REPORT(Asserter::format("Solved joints qh: (%s)",
                                     qh.subVector(3,dof.length()-1).toString(3,3).c_str()));
    RTF_TEST_REPORT(Asserter::format("Reached joints qf: (%s)",
                                     qf.subVector(0,6).toString(3,3).c_str()));
    RTF_TEST_REPORT(Asserter::format("Error: %g",compute_error(xh,oh,xf,of)));

    RTF_TEST_REPORT("Going back to starting pose");
    iarm->goToPoseSync(x,o);

    RTF_TEST_REPORT("Waiting");
    done=iarm->waitMotionDone(1.0,5.0);
    RTF_TEST_CHECK(done,"Starting pose reached!");

    RTF_TEST_REPORT("Cleaning up the context");
    iarm->restoreContext(context);
    iarm->deleteContext(context);
}

