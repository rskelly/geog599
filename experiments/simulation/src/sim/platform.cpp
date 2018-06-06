/*
 * platform.cpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#include <iostream>

#include <Eigen/Geometry>

#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"
#include "sim/gimbal.hpp"
#include "sim/terrain.hpp"

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

Eigen::Vector3d matrixToEuler(const Eigen::Matrix3d& mtx) {
	// TODO: Singularities
	Eigen::Vector3d vec;
	vec[0] = std::atan2(mtx(2, 1), mtx(2, 2));
	vec[1] = std::atan2(-mtx(2, 0), std::sqrt(std::pow(mtx(2,1), 2) + std::pow(mtx(2, 2), 2)));
	vec[2] = std::atan2(mtx(1, 0), mtx(0, 0));
	//std::cerr << "euler " << vec << "\n";
	return vec;
}

Eigen::Vector3d eulerToVector(const Eigen::Vector3d& euler) {
	Eigen::Matrix3d mtx = eulerToMatrix(euler);
	Eigen::Vector3d v(1, 0, 0);
	return mtx * v;
}

void printMatrix(const std::string& name, const Eigen::MatrixXd& mtx) {
	std::cerr << name << "\n" << mtx << "\n";
}

Platform::Platform() :
	m_forwardVelocity(10),
	m_lastTime(0),
	m_gimbal(nullptr),
	m_rangefinder(nullptr) {

	m_posPoisson.setMean(1000);
	m_rotPoisson.setMean(1000);

	m_position << 489103, 6502712, 320; // 30m high
	m_orientation << 0, 0, 0; // straight level
}


void Platform::update(double time) {
	double t = time - m_lastTime;
	m_lastTime = time;

	m_position[0] += t * m_posPoisson.next(m_forwardVelocity);
	m_position[1] += m_posPoisson.nextCentered();
	m_position[2] += m_posPoisson.nextCentered();

	m_orientation[0] += 0; //m_rotPoisson.nextCentered();
	m_orientation[1] += 0; //m_rotPoisson.nextCentered();
	m_orientation[2] += 0; //m_rotPoisson.nextCentered();

	// 1) Translate the laser into the gimbal's frame and rotate it using the gimbal's dynamic orientation. (The laser has no position info of its own).

	Eigen::Vector3d Gpd(m_gimbal->position());
	Eigen::Matrix3d God = eulerToMatrix(m_gimbal->orientation());
	Eigen::Vector3d A = God * Gpd;

	//std::cerr << "Gpd " << Gpd << "\n";
	//std::cerr << "God " << God << "\n";
	//std::cerr << "A " << A << "\n";

	// 2) Rotate the laser by the gimbal's static orientation and translate it into the platform's frame.

	Eigen::Vector3d Gps(m_gimbal->staticPosition());
	Eigen::Matrix3d Gos = eulerToMatrix(m_gimbal->staticOrientation());

	Eigen::Vector3d B = Gos * A + Gps;

	//std::cerr << "Gps " << Gps << "\n";
	//std::cerr << "Gos " << Gos << "\n";
	//std::cerr << "B " << B << "\n";

	// 3) Rotate the platform then translate into the inertial frame.

	Eigen::Vector3d Pp(position());
	Eigen::Matrix3d Po = eulerToMatrix(orientation());

	//std::cerr << "Pp " << Pp << "\n";
	//std::cerr << "Po " << Po << "\n";

	m_laserPosition = Po * B + Pp;

	// 4) Compute the laser orientation by adding the orientations. It passes through the laserPosition.

	//std::cerr << "laser orientation " << God * Gos * Po << "\n";

	m_laserDirection = eulerToVector(matrixToEuler(God * Gos * Po));

	//std::cerr << "Laser Position " << m_laserPosition << "\n";
	//std::cerr << "Laser Direction " << m_laserDirection << "\n";

	RangeBridge::setLaser(m_laserPosition, m_laserDirection);

	/*
	Range* range = m_rangefinder->range();

	if(range) {
		std::cerr << "Range " << range->range() << ", " << range->time() << "\n";
		std::cerr << "Position " << m_position[0] << ", " << m_position[1] << "\n";
		delete range;
	}
	*/
}

uav::Range* Platform::range() const {
	return m_rangefinder->range();
}

const Eigen::Vector3d Platform::laserPosition() const {
	return m_laserPosition;
}

const Eigen::Vector3d Platform::laserDirection() const {
	return m_laserDirection;
}

void Platform::setGimbal(uav::Gimbal* gimbal) {
	m_gimbal = gimbal;
}

void Platform::setRangefinder(uav::Rangefinder* rangefinder) {
	m_rangefinder = rangefinder;
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
