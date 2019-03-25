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

#ifndef _IKINIDYNCONSISTENCYTEST_H_
#define _IKINIDYNCONSISTENCYTEST_H_

#include <yarp/rtf/TestCase.h>

#include <yarp/sig/Matrix.h>

#include <iCub/iKin/iKinFwd.h>

namespace iCub
{
namespace iDyn
{
    class iCubWholeBody;
}
}

class iKiniDynConsistencyTest : public RTF::TestCase  {
private:
    yarp::sig::Vector q_head, q_torso, q_larm, q_rarm, q_lleg, q_rleg;
    iCub::iKin::iCubArm ikin_larm, ikin_rarm;
    iCub::iKin::iCubLeg ikin_lleg, ikin_rleg;
    iCub::iDyn::iCubWholeBody * icub;

public:
    iKiniDynConsistencyTest();
    virtual ~iKiniDynConsistencyTest();

    virtual bool setup(int argc, char** argv);

    virtual void tearDown();

    virtual void run();

    virtual yarp::sig::Matrix getiKinTransform(const std::string part,
                                               int index);

    virtual yarp::sig::Matrix getiDynTransform(const std::string part,
                                               int index);

    virtual void check_matrix_are_equal(const yarp::sig::Matrix & mat1,
                            const yarp::sig::Matrix & mat2,
                            double tol=1e-3);
};

#endif //_CAMERATEST_H
