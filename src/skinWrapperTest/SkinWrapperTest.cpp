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
#include <rtf/dll/Plugin.h>
#include <rtf/TestAssert.h>

#include "SkinWrapperTest.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;

// prepare the plugin
PREPARE_PLUGIN(SkinWrapperTest)


/***********************************************************************************/
SkinWrapperTest::SkinWrapperTest() :
                                       yarp::rtf::TestCase("SkinWrapperTest")
{
}


/***********************************************************************************/
SkinWrapperTest::~SkinWrapperTest()
{
}


/***********************************************************************************/
bool SkinWrapperTest::setup(Property &property)
{
    //test #1
    Property p;
    p.put("device","skinWrapper");
    p.put("robotName","fakeiCub");
    p.put("name","/testSkinWrapper");
    p.put("period",20);
    p.put("total_taxels",10);
    p.put("subdevice","fakeAnalogSensor");
    RTF_TEST_REPORT("Opening the skinWrapper with one port");
    RTF_TEST_CHECK(dd1.open(p),"Unable to open the device!");
    //test #2
    BufferedPort<Bottle> port,portrpc;
    portrpc.open("/fakeiCub2/skin/rpc:i");
    port.open("/fakeiCub2/skin");
    p.unput("robotName");
    p.put("robotName","fakeiCub2");
    RTF_TEST_REPORT("Opening the skinWrapper with one port (address conflict case)");
    RTF_TEST_CHECK(!dd2.open(p),"Unable to open the device as expected");
    portrpc.close();
    port.close();
    //test #3
    Property p2;
    p2.put("device","skinWrapper");
    p2.put("robotName","fakeiCub3");
    p2.put("name","/testSkinWrapperMultip");
    p2.put("period",20);
    p2.put("total_taxels",3);
    p2.put("subdevice","fakeAnalogSensor");
    p2.fromString("(ports (left_hand left_forearm left_arm)) (left_hand 0 0 0 0) (left_forearm 0 0 0 0) (left_arm 0 0 0 0)", false);
    RTF_TEST_REPORT("Opening the skinWrapper with multiple ports");
    RTF_TEST_CHECK(dd3.open(p2),"Unable to open the device!");
    //test #4
    port.open("/fakeiCub4/skin/left_hand");
    portrpc.open("/fakeiCub4/skin/left_forearm/rpc:i");
    p2.unput("robotName");
    p2.put("robotName","fakeiCub4");
    RTF_TEST_REPORT("Opening the skinWrapper with multiple ports (address conflict case)");
    RTF_TEST_CHECK(!dd4.open(p2),"Unable to open the device as expected");

    port.close();
    portrpc.close();
    return true;
}


/***********************************************************************************/
void SkinWrapperTest::tearDown()
{
    RTF_TEST_REPORT("Closing the skinWrapper");
    RTF_ASSERT_FAIL_IF_FALSE(dd1.close() && dd2.close() && dd3.close() && dd4.close(),"Unable to close the device!");
}


/***********************************************************************************/
void SkinWrapperTest::run()
{
    bool result;
    result  = Network::exists("/fakeiCub/skin/rpc:i");
    result &= Network::exists("/fakeiCub/skin");
    result &= Network::exists("/fakeiCub3/skin/left_hand");
    result &= Network::exists("/fakeiCub3/skin/left_hand/rpc:i");
    result &= Network::exists("/fakeiCub3/skin/left_forearm");
    result &= Network::exists("/fakeiCub3/skin/left_forearm/rpc:i");
    result &= Network::exists("/fakeiCub3/skin/left_arm");
    result &= Network::exists("/fakeiCub3/skin/left_arm/rpc:i");
    RTF_TEST_REPORT("Checking if all ports has been opened successfully");
    RTF_TEST_CHECK(result,"ports opened succefully");
    RTF_TEST_REPORT("Checking the validity of devices");
    RTF_TEST_CHECK(dd1.isValid() && !dd2.isValid() && dd3.isValid() && !dd4.isValid(),"dd1 and dd3 valid, dd2 and dd4 not valid as expected");


}

