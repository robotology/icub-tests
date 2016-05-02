// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Nuno GUedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _VALUEDISTRIBUTION_H_
#define _VALUEDISTRIBUTION_H_

#include <vector>
#include <array>


/**
 * \ingroup icub-tests
 * This class is not a YarpTestCase. It computes the min/max values and the distribution of values
 * added to a data set.
 *
 */

class ValueDistribution {
public:
    /**
     * Constructor
     *
     */
    ValueDistribution();
    ValueDistribution(unsigned int size,
                      double histIntervMin, double histIntervMax,
                      unsigned int bins);

    /**
     * Destructor
     *
     */
    virtual ~ValueDistribution();

    /**
     * Resize bins
     */
    virtual void resizeBins(double histIntervMin, double histIntervMax,
                            unsigned int bins);

    /**
     * Add a data point to the distribution and update distribution parameters
     */
    virtual void add(double elem);

    /**
     * Compute distribution parameters (mean, sigma, histogram.
     */
    virtual bool evalDistrParams();

    /**
     * Getters.
     */
    virtual double getElem(unsigned int index);

    typedef struct{
        double mean;
        double sigma;
        double min;
        double max;
        std::vector<double> hist;
    } distr_t;

    virtual distr_t getDistr();

private:
    std::vector<double> elemList;
    double mean;
    double sigma;
    double sumMean;
    double sumSigma;
    double min;
    double max;
    std::vector<double> hist;
    double histInterval[2];
    std::vector<double> binEdges;
};

#endif //_VALUEDISTRIBUTION_H_

