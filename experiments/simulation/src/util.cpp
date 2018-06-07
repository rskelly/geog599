/*
 * util.cpp
 *
 *  Created on: May 17, 2018
 *      Author: rob
 */

#include <iostream>
#include <sys/time.h>

#include "uav.hpp"
#include "util.hpp"

using namespace uav::util;

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

double Poisson::nextCentered(double freq) {
	int n = m_distribution(m_generator) - m_mean;
	return freq * (n / m_mean);
}

double uav::util::uavtime() {
	timeval time;
	gettimeofday(&time, NULL);
	return (double) time.tv_sec + ((double) time.tv_usec / 1000000);
}


static double r2d = PI / 180.0;
static double d2r = 180.0 / PI;

Eigen::Matrix3d uav::util::rotFromAxisAngle(const Eigen::Vector3d& vec, double angle) {
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

void uav::util::rotate(const Eigen::Vector3d& euler, Eigen::Vector3d& orientation) {
	double xc = std::cos(euler[0]), xs = std::sin(euler[0]);
	double yc = std::cos(euler[1]), ys = std::sin(euler[1]);
	double zc = std::cos(euler[2]), zs = std::sin(euler[2]);
	Eigen::Matrix3d x, y, z;
	x << 1, 0, 0, 0, xc, -xs, 0, xs, xc;
	y << yc, 0, ys, 0, 1, 0, -ys, 0, yc;
	z << zc, -zs, 0, zs, zc, 0, 0, 0, 1;
	//orientation *= z * y * x;
}

Eigen::Matrix3d uav::util::rotateX(double r) {
	double rc = std::cos(r), rs = std::sin(r);
	Eigen::Matrix3d mtx;
	mtx(0, 0) = 1;
	mtx(0, 1) = 0;
	mtx(0, 2) = 0;
	mtx(1, 0) = 0;
	mtx(1, 1) = rc;
	mtx(1, 2) =-rs;
	mtx(2, 0) = 0;
	mtx(2, 1) = rs;
	mtx(2, 2) = rc;
	return mtx;
}

Eigen::Matrix3d uav::util::rotateY(double r) {
	double rc = std::cos(r), rs = std::sin(r);
	Eigen::Matrix3d mtx;
	mtx(0, 0) = rc;
	mtx(0, 1) = 0;
	mtx(0, 2) = rs;
	mtx(1, 0) = 0;
	mtx(1, 1) = 1;
	mtx(1, 2) = 0;
	mtx(2, 0) = -rs;
	mtx(2, 1) = 0;
	mtx(2, 2) = rc;
	return mtx;
}

Eigen::Matrix3d uav::util::rotateZ(double r) {
	double rc = std::cos(r), rs = std::sin(r);
	Eigen::Matrix3d mtx;
	mtx(0, 0) = rc;
	mtx(0, 1) = -rs;
	mtx(0, 2) = 0;
	mtx(1, 0) = rs;
	mtx(1, 1) = rc;
	mtx(1, 2) = 0;
	mtx(2, 0) = 0;
	mtx(2, 1) = 0;
	mtx(2, 2) = 1;
	return mtx;
}

Eigen::Matrix3d uav::util::rotMatrix(double lat, double lon, double twist) {
	return  rotateZ(lat) * rotateY(lon) * rotateZ(twist);
}

Eigen::Matrix3d uav::util::eulerToMatrix(const Eigen::Vector3d& vec) {
	return rotMatrix(vec[0], vec[1], vec[2]);
}

Eigen::Vector3d uav::util::matrixToEuler(const Eigen::Matrix3d& mtx) {
	// TODO: Singularities
	Eigen::Vector3d vec;
	vec[0] = std::atan2(mtx(2, 1), mtx(2, 2));
	vec[1] = std::atan2(-mtx(2, 0), std::sqrt(std::pow(mtx(2,1), 2) + std::pow(mtx(2, 2), 2)));
	vec[2] = std::atan2(mtx(1, 0), mtx(0, 0));
	//std::cerr << "euler " << vec << "\n";
	return vec;
}

Eigen::Vector3d uav::util::eulerToVector(const Eigen::Vector3d& euler) {
	Eigen::Matrix3d mtx = eulerToMatrix(euler);
	Eigen::Vector3d v(1, 0, 0);
	return mtx * v;
}

void uav::util::printMatrix(const std::string& name, const Eigen::MatrixXd& mtx) {
	std::cerr << name << "\n" << mtx << "\n";
}

double uav::util::toRad(double deg) {
	return deg / d2r;
}

double uav::util::toDeg(double rad) {
	return rad / r2d;
}
