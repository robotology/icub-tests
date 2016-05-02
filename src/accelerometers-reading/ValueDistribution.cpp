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

#include "ValueDistribution.h"
#include <iDynTree/Core/EigenHelpers.h>

static std::vector<double> linspace(const double min, const int nSteps, const double max);

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
elemList(0,0),
sumMean(0),
sumSigma(0),
min(10),
max(0),
hist(bins,0)
{
    this->elemList.reserve(size);
    this->histInterval[0]=histIntervMin;
    this->histInterval[1]=histIntervMax;
    this->binEdges = linspace(this->histInterval[0], this->hist.size(), this->histInterval[1]);
}

void ValueDistribution::resizeBins(double histIntervMin, double histIntervMax,
                                   unsigned int bins)
{
    this->hist.resize(bins);
    this->hist.clear();
    this->histInterval[0]=histIntervMin;
    this->histInterval[1]=histIntervMax;
    this->binEdges = linspace(this->histInterval[0], this->hist.size(), this->histInterval[1]);
}

ValueDistribution::~ValueDistribution() {}

void ValueDistribution::add(double elem)
{
    int binIdx ;
    this->elemList.push_back(elem);
    this->sumMean += elem;
    this->sumSigma += pow(elem,2);
    this->min = fmin(this->min, elem);
    this->max = fmax(this->max, elem);

    // We define a distribution in the interval [9,11]
    for (binIdx=0; binIdx<binEdges.size(); binIdx++)
    {
        if (elem<binEdges[binIdx]) {break;}
    }
    // increment history for respective bin
    this->hist[binIdx]++;
}

bool ValueDistribution::evalDistrParams()
{
    // compute mean and standard deviation
    this->mean = this->sumMean/this->elemList.size();
    this->sigma = sqrt((this->sumSigma/this->elemList.size()) - pow(this->mean,2));
    // compute distribution
    Eigen::Map<Eigen::VectorXd>(this->hist.data(),this->hist.size()) =
    Eigen::Map<Eigen::VectorXd>(this->hist.data(),this->hist.size())/this->elemList.size();
    return true;
}

double ValueDistribution::getElem(unsigned int index)
{
    return this->elemList[index];
}

ValueDistribution::distr_t ValueDistribution::getDistr()
{
    distr_t params;
    params.mean = this->mean;
    params.sigma = this->sigma;
    params.min = this->min;
    params.max = this->max;
    params.hist = this->hist;
    return params;
}

static std::vector<double> linspace(const double min, const int nSteps, const double max)
{
    std::vector<double> linSpace(nSteps+1,0);
    double interv = (max-min)/nSteps;
    for(int idx=0; idx<(nSteps+1); idx++)
    {
        linSpace[idx] = min+idx*interv;
    }
    return linSpace;
}


