// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

//#include <cstdlib>
//#include <sstream>
//#include <string>
#include <vector>
#include <math.h>

#include "GravityEstimator.h"
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/HighLevel/DynamicsComputations.h>

using namespace iDynTree;
using namespace iDynTree::HighLevel;


GravityEstimator::GravityEstimator(std::string modelPath)
{
    /* Prepare inputs for updating the kinematics information in the estimator

     * Compute the kinematics information necessary for the accelerometer
     * sensor measurements estimation. We assume the robot root link is fixed to
     * the ground (steady kart pole). We then assume to know the gravity (ground
     * truth) projected on the frame (base_link) fixed to the root link. For more
     * info on iCub frames check: http://wiki.icub.org/wiki/ICub_Model_naming_conventions.

    obj.grav_idyn = iDynTree.Vector3();
    grav = [0.0;0.0;-9.81];
    obj.grav_idyn.fromMatlab(grav);

    // Create the estimator and model...
    %
    //Create an estimator class, load the respective model from URDF file and
    //set the robot state constant parameters

    //Create estimator class
    obj.estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();

    //Load model and sensors from the URDF file
    obj.estimator.loadModelAndSensorsFromFile(urdfModel);

    //Check if the model was correctly created by printing the model
    obj.estimator.model.toString()

    //Base link index for later applying forward kynematics
    //(specific to APPROACH 1)
    obj.base_link_index = obj.estimator.model.getFrameIndex('base_link');

    //Get joint information: DOF
    obj.dofs = obj.estimator.model.getNrOfDOFs();

    //create joint position iDynTree objects
    //Note: 'JointPosDoubleArray' is a special type for future evolution which
    //will handle quaternions. But for now the type has the format as
    //'JointDOFsDoubleArray'.
    obj.qi_idyn   = iDynTree.JointPosDoubleArray(obj.dofs);
    obj.dqi_idyn  = iDynTree.JointDOFsDoubleArray(obj.dofs);
    obj.d2qi_idyn = iDynTree.JointDOFsDoubleArray(obj.dofs);

    //Set the position of base link
    obj.fixedBasePos = iDynTree.FreeFloatingPos(obj.estimator.model);
    //           obj.fixedBasePos.worldBasePos = iDynTree.Transform.Identity();

    // Specify unknown wrenches (specific to APPROACH 1)
    //We need to set the location of the unknown wrench. We express the unknown
    //wrench at the origin of the l_sole frame
    unknownWrench = iDynTree.UnknownWrenchContact();
    unknownWrench.unknownType = iDynTree.FULL_WRENCH;

    //the position is the origin, so the conctact point wrt to base_link is zero
    unknownWrench.contactPoint.zero();

    //The fullBodyUnknowns is a class storing all the unknown external wrenches
    //acting on a class: we consider the pole reaction on the base link as the only
    //external force.
    //Build an empty list.
    obj.fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(obj.estimator.model());
    obj.fullBodyUnknowns.clear();
    obj.fullBodyUnknowns.addNewContactInFrame(obj.estimator.model, ...
                                              obj.base_link_index, ...
                                              unknownWrench);

    //Print the unknowns to make sure that everything is properly working
    obj.fullBodyUnknowns.toString(obj.estimator.model())


    // The estimated sensor measurements
    //`estimator.sensors()` gets used sensors (returns `SensorList`)
    //ex: `estimator.sensors.getNrOfSensors(iDynTree.ACCELEROMETER)`
    //    `estimator.sensors.getSensor(iDynTree.ACCELEROMETER,1)`
    obj.estMeasurements = iDynTree.SensorsMeasurements(obj.estimator.sensors);

    //Memory allocation for unused output variables
    obj.sink1 = iDynTree.LinkContactWrenches(obj.estimator.model);
    obj.sink2 = iDynTree.JointDOFsDoubleArray(obj.dofs);

    //estimation outputs
    obj.estimatedSensorLinAcc = iDynTree.LinearMotionVector3();

    //measurements
    obj.tmpSensorLinAcc = iDynTree.LinearMotionVector3();

    //full traversal for computing the base to link k transforms
    obj.traversal_Lk = iDynTree.Traversal();
    obj.linkPos = iDynTree.LinkPositions(obj.estimator.model);
*/
}

GravityEstimator::~GravityEstimator() {}

/**
 * Estimate Gravity measurements
 */
bool GravityEstimator::estimateGravityMeasurements(std::vector< std::vector<iDynTree::Vector3> >& sensorMeasMatList)
{
    /*
    % Fill iDynTree joint vectors.
    % Warning!! iDynTree takes in input **radians** based units,
    % while the iCub port stream **degrees** based units.
        qisRobotDOF = zeros(obj.dofs,1); qisRobotDOF(obj.jointsIdxListModel,1) = obj.q0i(:,ts);
    dqisRobotDOF = zeros(obj.dofs,1); dqisRobotDOF(obj.jointsIdxListModel,1) = obj.dqi(:,ts);
    d2qisRobotDOF = zeros(obj.dofs,1); d2qisRobotDOF(obj.jointsIdxListModel,1) = obj.d2qi(:,ts);
    obj.qi_idyn.fromMatlab(qisRobotDOF);
    obj.dqi_idyn.fromMatlab(dqisRobotDOF);
    obj.d2qi_idyn.fromMatlab(d2qisRobotDOF);

    % Update the kinematics information in the estimator
    obj.estimator.updateKinematicsFromFixedBase(obj.qi_idyn,obj.dqi_idyn,obj.d2qi_idyn, ...
                                                obj.base_link_index,obj.grav_idyn);

    % run the estimation
    obj.estimator.computeExpectedFTSensorsMeasurements(obj.fullBodyUnknowns,obj.estMeasurements,obj.sink1,obj.sink2);

    % Get predicted sensor data for each sensor referenced in 'sensorsIdxList'
        % and write them into the 'data' structure.
        for acc_i = 1:length(obj.sensorsIdxListModel)
            % get predicted measurement on sensor frame
            obj.estMeasurements.getMeasurement(iDynTree.ACCELEROMETER,obj.sensorsIdxListModel(acc_i),obj.estimatedSensorLinAcc);
    sensEst = obj.estimatedSensorLinAcc.toMatlab;
    % correction for MTB mounted upside-down
        if data.isInverted{obj.sensorsIdxListFile(acc_i)}
    sensEst = FrameConditioner.real_R_model*sensEst;
    end

    % get measurement table ys_xxx_acc [3xnSamples] from captured data,
    % and then select the sample 's' (<=> timestamp).
    ys   = ['ys_' data.labels{obj.sensorsIdxListFile(acc_i)}];
    eval(['data.parsedParams.' ys '(:,ts) = sensEst;']);
    end
     */
}


