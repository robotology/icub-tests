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

#include <string>
#include <robottestingframework/TestAssert.h>
#include <robottestingframework/dll/Plugin.h>
#include <yarp/os/Time.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include "CartesianControlReachingToleranceTest.h"

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

// prepare the plugin
ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(CartesianControlReachingToleranceTest)


/***********************************************************************************/
CartesianControlReachingToleranceTest::CartesianControlReachingToleranceTest() :
                                       yarp::robottestingframework::TestCase("CartesianControlReachingToleranceTest")
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

    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Opening Cartesian Controller Client for %s_arm",arm.c_str()));
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(drvCart.open(optCart),"Unable to open the client!");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Opening Joint Controller Client for %s_arm",arm.c_str()));
    ROBOTTESTINGFRAMEWORK_ASSERT_ERROR_IF_FALSE(drvJoint.open(optJoint),"Unable to open the client!");
    return true;
}


/***********************************************************************************/
void CartesianControlReachingToleranceTest::tearDown()
{
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Closing Cartesian Controller Client");
    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvCart.close(),"Unable to close the client!");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Closing Joint Controller Client");
    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL_IF_FALSE(drvJoint.close(),"Unable to close the client!");
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
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(drvCart.view(iarm),"Opening the view on the Cartesian device!");

    IEncoders *ienc;
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(drvJoint.view(ienc),"Opening the view on the Joint device!");

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
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Initial pose retrieved!");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Setting up the context");
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

    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Reaching for the target: (%s, %s)",
                                     pos2reach.toString(3,3).c_str(),ori2reach.toString(3,3).c_str()));
    Vector xh,oh,qh;
    iarm->goToPoseSync(pos2reach,ori2reach);
    iarm->getDesired(xh,oh,qh);

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Waiting");
    done=iarm->waitMotionDone(0.1,10.0);
    iarm->stopControl();

    Vector xf,of;
    iarm->getPose(xf,of);

    int nJoints;
    ienc->getAxes(&nJoints);
    Vector qf(nJoints);
    ienc->getEncoders(qf.data());

    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Target reached!");
    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Reaching tolerance: %g",tol));
    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Solved pose: (%s, %s)",
                                     xh.toString(3,3).c_str(),oh.toString(3,3).c_str()));
    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Reached pose: (%s, %s)",
                                     xf.toString(3,3).c_str(),of.toString(3,3).c_str()));
    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Solved joints qh: (%s)",
                                     qh.subVector(3,dof.length()-1).toString(3,3).c_str()));
    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Reached joints qf: (%s)",
                                     qf.subVector(0,6).toString(3,3).c_str()));
    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Error: %g",compute_error(xh,oh,xf,of)));

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Going back to starting pose");
    iarm->goToPoseSync(x,o);

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Waiting");
    done=iarm->waitMotionDone(1.0,5.0);
    ROBOTTESTINGFRAMEWORK_TEST_CHECK(done,"Starting pose reached!");

    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Cleaning up the context");
    iarm->restoreContext(context);
    iarm->deleteContext(context);
}

