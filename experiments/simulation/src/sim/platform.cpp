/*
 * platform.cpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#include <iostream>

#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"
#include "sim/gimbal.hpp"

#define PI 3.141592653589793238

using namespace uav::sim;

void rotate(const Eigen::Vector3d& euler, Eigen::Vector3d& orientation) {
	double xc = std::cos(euler[0]), xs = std::sin(euler[0]);
	double yc = std::cos(euler[1]), ys = std::sin(euler[1]);
	double zc = std::cos(euler[2]), zs = std::sin(euler[2]);
	Eigen::Matrix3d x, y, z;
	x << 1, 0, 0, 0, xc, -xs, 0, xs, xc;
	y << yc, 0, ys, 0, 1, 0, -ys, 0, yc;
	z << zc, -zs, 0, zs, zc, 0, 0, 0, 1;
	//orientation *= z * y * x;
}

Eigen::Matrix3d rotateX(double r) {
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

Eigen::Matrix3d rotateY(double r) {
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

Eigen::Matrix3d rotateZ(double r) {
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

Eigen::Matrix3d rotMatrix(double lat, double lon, double twist) {
	return  rotateZ(lat) * rotateY(lon) * rotateZ(twist);
}

Eigen::Matrix3d eulerToMatrix(const Eigen::Vector3d& vec) {
	return rotMatrix(vec[0], vec[1], vec[2]);
}

void printMatrix(const std::string& name, const Eigen::MatrixXd& mtx) {
	std::cerr << name << "\n" << mtx << "\n";
}


Platform::Platform() :
	m_forwardVelocity(10),
	m_lastTime(0),
	m_gimbal(new Gimbal()),
	m_rangefinder(new Rangefinder()) {

	m_gimbal->setStaticPosition(Eigen::Vector3d(0.2, 0, -0.05)); // 20cm forward, 0cm to side, 5cm down
	m_gimbal->setStaticOrientation(Eigen::Vector3d(0, 30 * PI / 180, 0)); // 30 rad down (around the y axis.)

	m_posPoisson.setMean(1000);
	m_rotPoisson.setMean(1000);

	m_position << 0, 0, 30; // 30m high
	m_orientation << 0, 0, 0; // straight level
}


void Platform::update(double time) {
	double t = time - m_lastTime;
	m_lastTime = time;

	m_position[0] += 0; //t * m_posPoisson.next(m_forwardVelocity);
	m_position[1] += 0; //m_posPoisson.nextCentered();
	m_position[2] += 0; //m_posPoisson.nextCentered();

	m_orientation[0] += 0; //m_rotPoisson.nextCentered();
	m_orientation[1] += 0; //m_rotPoisson.nextCentered();
	m_orientation[2] += 0; //m_rotPoisson.nextCentered();

	// TODO: Add some noisy movement to the orientation and position.
	//std::cerr << "position: " << m_position[0] << ", " << m_position[1] << ", " << m_position[2] << "\n";
	//std::cerr << "orientation: " << m_orientation[0] << ", " << m_orientation[1] << ", " << m_orientation[2] << "\n";

	// Need the laser pulse vector relative to the inertial frame.
	// 1) The laster pulse coordinate system is relative to the gimbal, offset by
    //    a static offset (Pl), rotated by a dynamic rotation (Rl).
	// 2) The gimbal coordinate system is relative to the platform frame
	//    offset by a static offset (Pg) and rotation (Rg).
	// 3) The platform frame is relative to the inertial frame, offset
	//    by a dynamic offset (Pp), rotated by a dynamic rotation (Rp).
	// To find the gimbal coordinate system relative to intertial frame:
	//     A = Pp + Pg * Rp
	// To find the laser coordinate system relative to the inertial frame:
	//     B = A + Pl * Rg
	// The pulse vector (C) rotates within its coordinate system by the
	// matrix Rl.

	Eigen::Vector3d Pp(m_position);
	Eigen::Vector3d Pgs(m_gimbal->staticPosition());
	Eigen::Matrix3d Rp = eulerToMatrix(m_orientation);

	//printMatrix("platform position", Pp);
	//printMatrix("gimbal static position", Pgs);
	//printMatrix("platform rotation", Rp);

	Eigen::Vector3d A = Pp + Rp * Pgs;

	//printMatrix("A", A);

	Eigen::Vector3d Pgd(m_gimbal->position());
	Eigen::Matrix3d Rg = eulerToMatrix(m_gimbal->staticOrientation());

	//printMatrix("gimbal dynamic position", Pgd);
	//printMatrix("gimbal static rotation", Rg);

	Eigen::Vector3d B = A + Rg * Pgd;

	//printMatrix("B", B);

	Eigen::Matrix3d Rgd = eulerToMatrix(m_gimbal->orientation());

	//printMatrix("gimbal dynamic rotation", Rgd);

	Eigen::Vector3d C = Rgd * B;

	std::cerr << "C: " << C[0] << ", " << C[1] << ", " << C[2] << "\n";
	//printMatrix("C", C);
}

const uav::Gimbal* Platform::gimbal() const {
	return m_gimbal;
}

const uav::Rangefinder* Platform::rangefinder() const {
	return m_rangefinder;
}

const Eigen::Vector3d& Platform::orientation() const {
	return m_orientation;
}

const Eigen::Vector3d& Platform::position() const {
	return m_position;
}

Platform::~Platform() {
	delete m_gimbal;
	delete m_rangefinder;
}
