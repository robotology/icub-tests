// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _GRAVITYESTIMATOR_H_
#define _GRAVITYESTIMATOR_H_

#include <vector>
#include <array>

#include <iDynTree/HighLevel/DynamicsComputations.h>

using namespace iDynTree;
using namespace iDynTree::HighLevel;

/**
 * \ingroup icub-tests
 * This class is not a YarpTestCase. It estimates the proper accelerations projected on the
 * inertial sensor frames.
 *
 */

class GravityEstimator {
public:
    /**
     * Constructor
     *
     */
    GravityEstimator(std::string modelPath);

    /**
     * Destructor
     *
     */
    virtual ~GravityEstimator();

    /**
     * Estimate Gravity measurements
     */
    virtual bool estimateGravityMeasurements(std::vector< std::vector<iDynTree::Vector3> >& sensorMeasMatList);

    /**
     * Getters.
     */

private:
    /*
    SpatialAcc grav_idyn;               // gravity iDynTree object
    int dofs;                    // joint information: DOF
    qi                 // joint position iDynTree object
    dqi_idyn                // joint velocity iDynTree object
    d2qi_idyn               // joint acceleration iDynTree object
    estimator               // estimator for computing the estimated sensor measurements
    base_link_index         // input param of estimator. iDynTree model indexing
    fullBodyUnknowns        // input param of estimator
    estMeasurements         // input param of estimator
    sink1                   // sink for output estContactForces
    sink2                   // sink for output estJointTorques
    sensorsIdxListModel = []; // subset of active sensors: indices from iDynTree model
    sensorsIdxListFile  = []; // subset of active sensors: indices from 'data.frame' list,
    %  ordered as per the data.log format.
    jointsLabelIdx = 0;         // index of 'StateExt' in 'data.frame' list
    jointsIdxListModel  = [];   // map 'joint to calibrate' to iDynTree joint index
    estimatedSensorLinAcc       // predicted measurement on sensor frame
    tmpSensorLinAcc              // sensor measurement
    q0i                         // joint positions for the current processed part.
    dqi                         // joint velocities for the current processed part.
    d2qi                        // joint accelerations for the current processed part.
    DqiEnc                      // vrtual joint offsets from the encoders.
                                // specific to APPROACH 2: measurements projected on each link
    traversal_Lk                // full traversal for computing the link positions
    fixedBasePos                // full tree joint positions (including base link)
                                // (required by the estimator interface,
                                // but the base position is not really
                                // relevant for computing the transforms
                                // between segment frames).
    linkPos                     // link positions w.r.t. the chosen base (base="projection link")



    std::vector<double> elemList;
    double mean;
    double sigma;
    double sumMean;
    double sumSigma;
    double min;
    double max;
    std::vector<double> hist;
    double histInterval[2];
    std::vector<double> binEdges;
     */
};

#endif //_GRAVITYESTIMATOR_H_

