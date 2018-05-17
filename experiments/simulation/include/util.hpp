/*
 * util.hpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#ifndef INCLUDE_UTIL_HPP_
#define INCLUDE_UTIL_HPP_

#include <random>

/**
 * Provides a simple method for retrieving a poisson-distributed
 * value given a mean and a frequency.
 */
class Poisson {
private:
	double m_mean;
	std::default_random_engine m_generator;
	std::poisson_distribution<int> m_distribution;

public:

	Poisson();

	Poisson(double mean);

	void setMean(double mean);

	double next(double freq = 1.0);

	double nextCentred(double freq = 1.0);

};



#endif /* INCLUDE_UTIL_HPP_ */
