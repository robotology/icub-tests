// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
