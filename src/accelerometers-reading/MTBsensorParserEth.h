// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _MTBSENSORPARSERETH_H_
#define _MTBSENSORPARSERETH_H_

#include <string>
#include <vector>
#include <map>
#include <yarp/sig/Vector.h>
#include "IMTBsensorParser.h"


/**
 * \ingroup icub-tests
 */

class MTBsensorParserEth : public IMTBsensorParser {
public:
    /**
     * Constructor
     *
     */
    MTBsensorParserEth();

    /**
     * Denstructor
     *
     */
    virtual ~MTBsensorParserEth();

    /**
     * Checks the format and size of the sensor data read from a the YARP port,
     * matches the provided sensor list to the actual sensor IDs published on the YARP port,
     * computes the mapping between the sensor list and data offsets in the data stream.
     */
    virtual bool mapSensorData(yarp::sig::Vector *readSensor,
                               std::vector<sensorTypeT> typeList, yarp::os::Bottle &sensorList,
                               yarp::os::Bottle &availSensorList, std::string &errorMsg);

    /**
     * Returns the sensor data as a vector of measurements
     *
     */
    virtual void parseSensorMeas(yarp::sig::Vector * readSensor,
                                 std::vector< std::array<double,3> > &sensorMeasList);

    /**
     * Checks that control data didn't change
     *
     */
    virtual bool checkControlData(yarp::sig::Vector * readSensor);

private:
    /*
     * ==== MTB Data format ====:
     *
     * a = (n  6.0  (a1  b1 t1 x1 y1 x1)   .... (an  bn tn xn yn xn))
     * ai = pos of sensor ... see enum type
     * bi = accel (1) or gyro (2)
     * tn = time stamp.
     *
     */

    // Below enums are directly copied from icub-firmware-shared/eth/embobj/plus/comm-v2/icub/EoAnalogSensors.h

    // Matches sensorTypeT
    typedef enum
    {
        eoas_inertial_type_none          = 0,
        eoas_inertial_type_accelerometer = 1,
        eoas_inertial_type_gyroscope     = 2
    } eOas_inertial_type_t;

    enum { eoas_inertial_pos_max_numberof = 63 };

    enum { eoas_inertial_pos_offsetleft = 0, eoas_inertial_pos_offsetright = 24, eoas_inertial_pos_offsetcentral = 48 };

    /** @typedef    typedef enum eOas_inertial_position_t
     @brief      contains a unique id for every possible inertial sensor positioned on iCub. So far we can host
     up to 63 different positions. The actual positions on iCub are documented on http://wiki.icub.org/wiki/Distributed_Inertial_sensing
     where one must look for the tags 10B12, 10B13 etc. The mapping on CAN for the ETH robot v3 is written aside.
     **/
   typedef enum
    {
        eoas_inertial_pos_none                  = 0,

        // left arm
        eoas_inertial_pos_l_hand                = 1+eoas_inertial_pos_offsetleft,       // label 1B7    canloc = (CAN2, 14)
        eoas_inertial_pos_l_forearm_1           = 2+eoas_inertial_pos_offsetleft,       // label 1B8    canloc = (CAN2, 12)
        eoas_inertial_pos_l_forearm_2           = 3+eoas_inertial_pos_offsetleft,       // label 1B9    canloc = (CAN2, 13)
        eoas_inertial_pos_l_upper_arm_1         = 4+eoas_inertial_pos_offsetleft,       // label 1B10   canloc = (CAN2,  9)
        eoas_inertial_pos_l_upper_arm_2         = 5+eoas_inertial_pos_offsetleft,       // label 1B11   canloc = (CAN2, 11)
        eoas_inertial_pos_l_upper_arm_3         = 6+eoas_inertial_pos_offsetleft,       // label 1B12   canloc = (CAN2, 10)
        eoas_inertial_pos_l_upper_arm_4         = 7+eoas_inertial_pos_offsetleft,       // label 1B13   canloc = (CAN2,  8)
        // left leg
        eoas_inertial_pos_l_foot_1              = 8+eoas_inertial_pos_offsetleft,       // label 10B12  canloc = (CAN2, 13)
        eoas_inertial_pos_l_foot_2              = 9+eoas_inertial_pos_offsetleft,       // label 10B13  canloc = (CAN2, 12)
        eoas_inertial_pos_l_lower_leg_1         = 10+eoas_inertial_pos_offsetleft,      // label 10B8   canloc = (CAN2,  8)
        eoas_inertial_pos_l_lower_leg_2         = 11+eoas_inertial_pos_offsetleft,      // label 10B9   canloc = (CAN2,  9)
        eoas_inertial_pos_l_lower_leg_3         = 12+eoas_inertial_pos_offsetleft,      // label 10B10  canloc = (CAN2, 10)
        eoas_inertial_pos_l_lower_leg_4         = 13+eoas_inertial_pos_offsetleft,      // label 10B11  canloc = (CAN2, 11)
        eoas_inertial_pos_l_upper_leg_1         = 14+eoas_inertial_pos_offsetleft,      // label 10B1   canloc = (CAN1,  1)
        eoas_inertial_pos_l_upper_leg_2         = 15+eoas_inertial_pos_offsetleft,      // label 10B2   canloc = (CAN1,  2)
        eoas_inertial_pos_l_upper_leg_3         = 16+eoas_inertial_pos_offsetleft,      // label 10B3   canloc = (CAN1,  3)
        eoas_inertial_pos_l_upper_leg_4         = 17+eoas_inertial_pos_offsetleft,      // label 10B4   canloc = (CAN1,  4)
        eoas_inertial_pos_l_upper_leg_5         = 18+eoas_inertial_pos_offsetleft,      // label 10B5   canloc = (CAN1,  5)
        eoas_inertial_pos_l_upper_leg_6         = 19+eoas_inertial_pos_offsetleft,      // label 10B6   canloc = (CAN1,  6)
        eoas_inertial_pos_l_upper_leg_7         = 20+eoas_inertial_pos_offsetleft,      // label 10B7   canloc = (CAN1,  7)

        // right arm
        eoas_inertial_pos_r_hand                = 1+eoas_inertial_pos_offsetright,      // label 2B7    canloc = (CAN2, 14)
        eoas_inertial_pos_r_forearm_1           = 2+eoas_inertial_pos_offsetright,      // label 2B8    canloc = (CAN2, 12)
        eoas_inertial_pos_r_forearm_2           = 3+eoas_inertial_pos_offsetright,      // label 2B9    canloc = (CAN2, 13)
        eoas_inertial_pos_r_upper_arm_1         = 4+eoas_inertial_pos_offsetright,      // label 2B10   canloc = (CAN2,  9)
        eoas_inertial_pos_r_upper_arm_2         = 5+eoas_inertial_pos_offsetright,      // label 2B11   canloc = (CAN2, 11)
        eoas_inertial_pos_r_upper_arm_3         = 6+eoas_inertial_pos_offsetright,      // label 2B12   canloc = (CAN2, 10)
        eoas_inertial_pos_r_upper_arm_4         = 7+eoas_inertial_pos_offsetright,      // label 2B13   canloc = (CAN2,  8)
        // right leg
        eoas_inertial_pos_r_foot_1              = 8+eoas_inertial_pos_offsetright,      // label 11B12  canloc = (CAN2, 13)
        eoas_inertial_pos_r_foot_2              = 9+eoas_inertial_pos_offsetright,      // label 11B13  canloc = (CAN2, 12)
        eoas_inertial_pos_r_lower_leg_1         = 10+eoas_inertial_pos_offsetright,     // label 11B8   canloc = (CAN2,  8)
        eoas_inertial_pos_r_lower_leg_2         = 11+eoas_inertial_pos_offsetright,     // label 11B9   canloc = (CAN2,  9)
        eoas_inertial_pos_r_lower_leg_3         = 12+eoas_inertial_pos_offsetright,     // label 11B10  canloc = (CAN2, 10)
        eoas_inertial_pos_r_lower_leg_4         = 13+eoas_inertial_pos_offsetright,     // label 11B11  canloc = (CAN2, 11)
        eoas_inertial_pos_r_upper_leg_1         = 14+eoas_inertial_pos_offsetright,     // label 11B1   canloc = (CAN1,  1)
        eoas_inertial_pos_r_upper_leg_2         = 15+eoas_inertial_pos_offsetright,     // label 11B2   canloc = (CAN1,  2)
        eoas_inertial_pos_r_upper_leg_3         = 16+eoas_inertial_pos_offsetright,     // label 11B3   canloc = (CAN1,  3)
        eoas_inertial_pos_r_upper_leg_4         = 17+eoas_inertial_pos_offsetright,     // label 11B5   canloc = (CAN1,  5)
        eoas_inertial_pos_r_upper_leg_5         = 18+eoas_inertial_pos_offsetright,     // label 11B4   canloc = (CAN1,  4)
        eoas_inertial_pos_r_upper_leg_6         = 19+eoas_inertial_pos_offsetright,     // label 11B6   canloc = (CAN1,  6)
        eoas_inertial_pos_r_upper_leg_7         = 20+eoas_inertial_pos_offsetright,     // label 11B7   canloc = (CAN1,  7)

        // central parts
        eoas_inertial_pos_chest_1               = 1+eoas_inertial_pos_offsetcentral,    // 9B7
        eoas_inertial_pos_chest_2               = 2+eoas_inertial_pos_offsetcentral,    // 9B8
        eoas_inertial_pos_chest_3               = 3+eoas_inertial_pos_offsetcentral,    // 9B9
        eoas_inertial_pos_chest_4               = 4+eoas_inertial_pos_offsetcentral,    // 9B10

        eOas_inertial_pos_jolly_1               = 60,
        eOas_inertial_pos_jolly_2               = 61,
        eOas_inertial_pos_jolly_3               = 62,
        eOas_inertial_pos_jolly_4               = 63
        
    } eOas_inertial_position_t;

    // LUT of MTB IDs indexed by the MTB enum defined above.
    const std::string LUTmtbEnum2Id[1+eoas_inertial_pos_max_numberof] = {"",
        "1B7","1B8","1B9","1B10","1B11","1B12","1B13",
        "10B12","10B13","10B8","10B9","10B10","10B11","10B1","10B2","10B3","10B4","10B5","10B6","10B7",
        "","","","",
        "2B7","2B8","2B9","2B10","2B11","2B12","2B13",
        "11B12","11B13","11B8","11B9","11B10","11B11","11B1","11B2","11B3","11B5","11B4","11B6","11B7",
        "","","","",
        "9B7","9B8","9B9","9B10",
        "","","",""};

    const double version = 6.0;
    yarp::sig::Vector rawSensorConfig;
    std::map<std::string,int> availMTBsensIDs;
    std::vector<int> reqMTBsensOffsets;

};
#endif //_MTBSENSORPARSERETH_H_


