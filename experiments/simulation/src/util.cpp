/*
 * util.cpp
 *
 *  Created on: May 17, 2018
 *      Author: rob
 */

#include "util.hpp"


Poisson::Poisson() : Poisson(10) {}

Poisson::Poisson(double mean) {
	setMean(mean);
}

void Poisson::setMean(double mean) {
	m_mean = mean;
	m_distribution.param(std::poisson_distribution<int>::param_type{mean});
}

double Poisson::next(double freq) {
	int n = m_distribution(m_generator);
	return freq * (n / m_mean);
}

double Poisson::nextCentred(double freq) {
	int n = m_distribution(m_generator) - m_mean;
	return freq * (n / m_mean);
}
