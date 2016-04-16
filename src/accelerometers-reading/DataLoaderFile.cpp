// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

//#include <cstdlib>
#include <string>
#include <sstream>
//#include <vector>
//#include <array>
//
#include "DataLoaderFile.h"
#include "yarp/os/Property.h"
#include "yarp/sig/Vector.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

DataLoaderFile::DataLoaderFile() {}

DataLoaderFile::~DataLoaderFile() {}

bool DataLoaderFile::setup(Property& configuration, std::string &statusMsg)
{
    // open log file
    string logFileName = configuration.find("dataDumpFile").asString();
    this->dataDumpFileStr.open(logFileName.c_str(), fstream::out);

    // check if file is open
    if(!this->dataDumpFileStr.is_open())
    {
        statusMsg = "Failed to open data dump file";
        return false;
    }
    else{
        statusMsg = "Data dump file opened successfuly";
        return true;
    }
}

void DataLoaderFile::tearDown()
{
    if(this->dataDumpFileStr.is_open()) {this->dataDumpFileStr.close();}
}

Vector* DataLoaderFile::read()
{
    /*
     * Read data from log file.
     */
    // read one string line from file
    string lineFromFile;
    getline(this->dataDumpFileStr, lineFromFile);
//    cout << "lineFromFile:\n" << lineFromFile.c_str() << endl;
//    Bottle lineSerialized = Bottle(ConstString(lineFromFile));
//    cout << "bottle:\n" << lineSerialized.toString().c_str() << endl;
//    lineSerialized.write(this->yarpVecFromFile,false);

    // separate the elements of the line (parameters)
    istringstream iss; iss.str(lineFromFile);
    double wordVal;
    // Discard first two elements (index and global timestamp)
    iss >> wordVal; iss >> wordVal;

    // Read remaining elements until End Of Line
    yarpVecFromFile.clear();
    while(!iss.eof())
    {
        iss >> wordVal;
        this->yarpVecFromFile.push_back(wordVal);
    }

    return &yarpVecFromFile;
}

void DataLoaderFile::delayBeforeRead()
{
}
