/*
 * gimbal.cpp
 *
 *  Created on: May 25, 2018
 *      Author: rob
 */

#include <chrono>
#include <iostream>

#include <boost/asio.hpp>

#include <Eigen/Core>

#include "uav.hpp"
#include "util.hpp"
#include "sim/gimbal.hpp"

using namespace uav::sim;
using namespace uav::util;

constexpr double G_CLOCK_DELAY = 1.0 / 10000.0;

SinGimbal::SinGimbal(double sweepAngle, double sweepFrequency) :
	m_orientation(Eigen::Vector3d(0, 0, 0)),
	m_position(Eigen::Vector3d(0, 0, 0)),
	m_staticOrientation(Eigen::Vector3d(0, 0, 0)),
	m_staticPosition(Eigen::Vector3d(0, 0, 0)),
	m_running(false),
	m_sweepAngle(sweepAngle / 2), m_sweepFrequency(sweepFrequency) {
}

void SinGimbal::setSweepAngle(double sweepAngle) {
	m_sweepAngle = sweepAngle / 2;
}

void SinGimbal::setSweepFrequency(double sweepFrequency) {
	m_sweepFrequency = sweepFrequency;
}

void SinGimbal::tick(double time) {
	double a = std::sin(time * PI * 2 * m_sweepFrequency) * m_sweepAngle;
	m_orientation[2] = a; // around z-axis (side to side)
}

void SinGimbal::start() {
	Clock::addObserver(this, G_CLOCK_DELAY);
}

void SinGimbal::stop() {
	Clock::removeObserver(this);
}

void SinGimbal::setOrientation(const Eigen::Vector3d& orientation) {
	m_orientation = orientation;
}

const Eigen::Vector3d& SinGimbal::orientation() const {
	return m_orientation;
}

void SinGimbal::setPosition(const Eigen::Vector3d& position) {
	m_position = position;
}

const Eigen::Vector3d& SinGimbal::position() const {
	return m_position;
}

void SinGimbal::setStaticOrientation(const Eigen::Vector3d& mtx) {
	m_staticOrientation = mtx;
}

const Eigen::Vector3d& SinGimbal::staticOrientation() const {
	return m_staticOrientation;
}

void SinGimbal::setStaticPosition(const Eigen::Vector3d& mtx) {
	m_staticPosition = mtx;
}

const Eigen::Vector3d& SinGimbal::staticPosition() const {
	return m_staticPosition;
}

SinGimbal::~SinGimbal() {
	stop();
}


using boost::asio::ip::udp;

void _netrun(const std::string* addr, int port, Eigen::Vector3d* orientation, bool* running) {

	boost::asio::io_service svc;
	udp::resolver res(svc);
	udp::resolver::query query(udp::v4(), *addr, std::to_string(port));
	udp::endpoint endpoint = *res.resolve(query);

	udp::socket sock(svc);
	sock.open(udp::v4());

	std::vector<char> send(1);
	sock.send_to(boost::asio::buffer(send), endpoint);

	std::vector<char> recv(128);
	udp::endpoint sender;
	while(*running) {
		size_t len = sock.receive_from(boost::asio::buffer(recv), sender);
		for(size_t i = 0; i < len; i += 8) {
			double angle = *((double*) recv.data() + i) * PI / 180.0;
			(*orientation)[2] = angle;
		}
		std::this_thread::yield();
	}

}

NetGimbal::NetGimbal(const std::string& addr, int port) :
	m_addr(addr),
	m_port(port),
	m_running(false) {
}

void NetGimbal::start() {
	if(!m_running) {
		m_running = true;
		m_thread = std::thread(_netrun, &m_addr, m_port, &m_orientation, &m_running);
	}
}

void NetGimbal::stop() {
	if(m_running) {
		m_running = false;
		if(m_thread.joinable())
			m_thread.join();
	}
}

void NetGimbal::setOrientation(const Eigen::Vector3d& orientation) {
	m_orientation = orientation;
}

const Eigen::Vector3d& NetGimbal::orientation() const {
	return m_orientation;
}

void NetGimbal::setPosition(const Eigen::Vector3d& position) {
	m_position = position;
}

const Eigen::Vector3d& NetGimbal::position() const {
	return m_position;
}

void NetGimbal::setStaticOrientation(const Eigen::Vector3d& mtx) {
	m_staticOrientation = mtx;
}

const Eigen::Vector3d& NetGimbal::staticOrientation() const {
	return m_staticOrientation;
}

void NetGimbal::setStaticPosition(const Eigen::Vector3d& mtx) {
	m_staticPosition = mtx;
}

const Eigen::Vector3d& NetGimbal::staticPosition() const {
	return m_staticPosition;
}

void NetGimbal::tick(double tick) {

}

NetGimbal::~NetGimbal() {

}
