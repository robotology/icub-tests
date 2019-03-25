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

#include <rtf/dll/Plugin.h>
#include <yarp/os/Port.h>
#include <yarp/os/SystemInfoSerializer.h>
#include "SystemStatus.h"

using namespace std;
using namespace RTF;
using namespace yarp::os;

#define CONNECTION_TIMEOUT      5.0         //seconds

// prepare the plugin
PREPARE_PLUGIN(SystemStatus)


SystemStatus::SystemStatus() : yarp::rtf::TestCase("SystemStatus") {
}

SystemStatus::~SystemStatus() { }

bool SystemStatus::setup(yarp::os::Property &property) {

    //updating the test name
    if(property.check("name"))
        setName(property.find("name").asString());

    RTF_ASSERT_ERROR_IF(property.check("hosts"),
                        "A list of hosts name must be given using 'hosts' param");
    yarp::os::Bottle portsSet = property.findGroup("hosts").tail();
    for(unsigned int i=0; i<portsSet.size(); i++) {
        yarp::os::Bottle* btport = portsSet.get(i).asList();
        RTF_ASSERT_ERROR_IF(btport && btport->size()>=2, "Hosts must be given as lists of <host name> <max cpu load>");
        HostInfo info;
        info.name = btport->get(0).asString();
        info.maxCpuLoad = btport->get(1).asInt();
        hosts.push_back(info);
    }

    return true;
}

void SystemStatus::tearDown() {
    // finalization goes her ...
}

void SystemStatus::run() {
    for(unsigned int i=0; i<hosts.size(); i++) {
        SystemInfoSerializer info;
        RTF_TEST_REPORT("");
        RTF_TEST_REPORT(Asserter::format("Checking host %s ...", hosts[i].name.c_str()));
        bool ret = getSystemInfo(hosts[i].name, info);
        RTF_TEST_FAIL_IF(ret, Asserter::format("Failed to get the system status of host %s. Is the yarprun running on %s?",
                                             hosts[i].name.c_str(), hosts[i].name.c_str()));
        if(ret) {
            RTF_TEST_REPORT(Asserter::format("Total memory %d MB, free memory %d MB.",
                                             info.memory.totalSpace, info.memory.freeSpace));
            unsigned int cpuLoad1 = (int)(info.load.cpuLoad1*100);
            unsigned int cpuLoad5 = (int)(info.load.cpuLoad5*100);
            unsigned int cpuLoad15 = (int)(info.load.cpuLoad15*100);
            RTF_TEST_REPORT(Asserter::format("Cpu load during last minute %d\%, last 5 minutes %d\%, last 15 minutes %d\%",
                                             cpuLoad1, cpuLoad5, cpuLoad15));
            RTF_TEST_FAIL_IF(cpuLoad1 < hosts[i].maxCpuLoad,
                           Asserter::format("Cpu load (last minute) is higher than desired [%d\%]", hosts[i].maxCpuLoad));
        }
    }
}

bool SystemStatus::getSystemInfo(std::string remote,
                                 SystemInfoSerializer& info) {

    yarp::os::Port port;

    // opening the port
    port.setTimeout(CONNECTION_TIMEOUT);
    RTF_ASSERT_ERROR_IF(port.open("..."), "Cannot open the yarp port");

    yarp::os::Bottle msg, grp;
    grp.clear();
    grp.addString("sysinfo");
    msg.addList() = grp;

    ContactStyle style;
    style.quiet = true;
    style.timeout = CONNECTION_TIMEOUT;

    bool connected = yarp::os::NetworkBase::connect(port.getName(), remote.c_str(), style);
    if(!connected) {
        RTF_TEST_FAIL_IF(connected, string("Cannot connect to ") + remote);
        port.close();
        return false;
    }

    bool ret = port.write(msg, info);
    NetworkBase::disconnect(port.getName().c_str(), remote.c_str());
    if(!ret) {
        port.close();
        RTF_TEST_FAIL_IF(ret, remote + string(" does not respond"));
        return false;
    }

    port.close();
    return true;
}

