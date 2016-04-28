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
//#include <vector>
//#include <array>
#include <math.h>

#include "ValueDistribution.h"

//using namespace std;

ValueDistribution::ValueDistribution() :
sumMean(0),
sumSigma(0),
min(10),
max(0),
hist(50,0)
{
    this->histInterval[0]=0;
    this->histInterval[1]=0;
}

ValueDistribution::ValueDistribution(unsigned int size,
                                     double histIntervMin, double histIntervMax,
                                     unsigned int bins) :
sumMean(0),
sumSigma(0),
min(10),
max(0),
hist(bins,0)
{
    this->elemList.reserve(size);
    this->histInterval[0]=histIntervMin;
    this->histInterval[1]=histIntervMax;
}

void ValueDistribution::resizeBins(double histIntervMin, double histIntervMax,
                                   unsigned int bins)
{
    this->hist.resize(bins);
    this->hist.clear();
    this->histInterval[0]=histIntervMin;
    this->histInterval[1]=histIntervMax;
}

ValueDistribution::~ValueDistribution() {}

void ValueDistribution::add(double elem)
{
    elemList.push_back(elem);
    this->sumMean += elem;
    this->sumSigma += pow(elem,2);
    this->min = fmin(this->min, elem);
    this->max = fmax(this->max, elem);
    // Derive bin associated to 'elem'.
    // We define a distribution in the interval [9,11]

    int idx = 0;
    // increment history for respective bin
    this->hist[idx]++;
}

bool ValueDistribution::evalDistrParams()
{
    this->mean = this->sumMean/this->elemList.size();
    this->sigma = sqrt((this->sumSigma/this->elemList.size()) - pow(this->mean,2));
    return true;
}

double ValueDistribution::getElem(unsigned int index)
{
    return this->elemList[index];
}

double ValueDistribution::getMean()
{
    return this->mean;
}

double ValueDistribution::getSigma()
{
    return this->sigma;
}

