// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _SYSTEM_STATUS_H_
#define _SYSTEM_STATUS_H_

#include <vector>
#include <yarp/rtf/TestCase.h>
#include <yarp/os/SystemInfoSerializer.h>

class HostInfo {
public:
    std::string name;
    int maxCpuLoad;
};


class SystemStatus : public yarp::rtf::TestCase {
public:
    SystemStatus();
    virtual ~SystemStatus();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();

private:
    bool getSystemInfo(std::string remote,
                       yarp::os::SystemInfoSerializer& info);
private:
    std::vector<HostInfo> hosts;
};

#endif //_SYSTEM_STATUS

