// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _YarpFixManager_H_
#define _YarpFixManager_H_

#include <yarp/os/Network.h>
#include <yarp/manager/manager.h>
#include <FixtureManager.h>

class YarpFixManager : public RTF::FixtureManager,
        yarp::manager::Manager {
public:
    virtual bool setup(int argc, char** argv);

    virtual void tearDown();

protected:
    virtual void onExecutableStart(void* which);
    virtual void onExecutableStop(void* which);
    virtual void onExecutableDied(void* which);
    virtual void onExecutableFailed(void* which);
    virtual void onCnnStablished(void* which);
    virtual void onCnnFailed(void* which);

private:
    yarp::os::Network yarp;
};

#endif //_YarpFixManager_H_
