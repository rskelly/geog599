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


#define PI 3.141592653589793

static double r2d = PI / 180.0;
static double d2r = 180.0 / PI;

Eigen::Matrix3d MatrixUtil::rotFromAxisAngle(const Eigen::Vector3d& vec, double angle) {
	Eigen::Vector3d norm(vec);
	norm.normalize();

	Eigen::Matrix3d out;

	double cosa = std::cos(angle);
	double sina = std::sin(angle);
	double nx = norm[0], ny = norm[1], nz = norm[2];
	double nx2 = nx * nx, ny2 = ny * ny, nz2 = nz * nz;

	out << (cosa + nx2 * (1.0 - cosa)), (nx * ny * (1.0 - cosa) - nz * sina), (nx * nz * (1.0 - cosa) + ny * sina),
			(ny * nx * (1.0 - cosa) + nz * sina), (cosa + ny2 * (1.0 - cosa)), (ny * nz * (1.0 - cosa) - nx * sina),
			(nz * nx * (1.0 - cosa) - ny * sina), (nz * ny * (1.0 - cosa) + nx * sina), (cosa + nz2 * (1.0 - cosa));

	return out;
}

double MatrixUtil::toRad(double deg) {
	return deg / d2r;
}

double MatrixUtil::toDeg(double rad) {
	return rad / r2d;
}
