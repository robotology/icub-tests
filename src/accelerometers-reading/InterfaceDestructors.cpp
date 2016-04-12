// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

/**
 * \file ModelInterfaceDestructors.cpp
 *
 * For a strange feature of C++, all interfaces (i.e. pure abstract classes)
 * require a destructor body for their destructor, even if it abstract.
 *
 * This file contains this "dummy" destructors for all the pure abstract classes of iDynTree Model.
 *
 */

#include "IMTBsensorParser.h"

IMTBsensorParser::~IMTBsensorParser() {}
