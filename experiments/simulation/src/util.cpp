/*
 * util.cpp
 *
 *  Created on: May 17, 2018
 *      Author: rob
 */

#include <iostream>
#include <chrono>

#include "uav.hpp"
#include "util.hpp"

void uav::thread::setPriority(std::thread &th, int policy, int priority) {
	sched_param sch_params;
	sch_params.sched_priority = priority;
	if(pthread_setschedparam(th.native_handle(), policy, &sch_params))
		std::cerr << "Failed to set thread scheduling : " << std::strerror(errno) << "\n";
}

void uav::thread::setAffinity(std::thread &th, int core) {
	pthread_t thread = th.native_handle();
	cpu_set_t cpuset;

	CPU_ZERO(&cpuset);
	CPU_SET(core, &cpuset);

	if(pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset))
		std::cerr << "Failed to set thread affinity: " << std::strerror(errno) << "\n";
}

using namespace uav::util;

ClockObserverItem::ClockObserverItem(ClockObserver* item, double delay) :
		delay(delay), lastTick(-1), item(item)  {}

void __clockRun(std::list<ClockObserverItem>* observers, bool* running, double* currentTime, double* minStep) {
	double time;

	// Set the initial time on all observer items.
	time = 0; // uavtime(); No need to start at UTC. Maybe can add another field later.
	for(ClockObserverItem& item : *observers)
		item.lastTick = time;

	while(*running) {
		time += *minStep;
		*currentTime = time;
		// Iterate over observer items, firing those whose tick delay has elapsed.
		for(ClockObserverItem& item : *observers) {
			if(item.delay <= time - item.lastTick) {
				item.item->tick(time);
				item.lastTick = time;
			}
		}
		std::this_thread::sleep_for(std::chrono::nanoseconds((int) (*minStep * 1000000000)));
	}
}

Clock::Clock() :
	m_running(false),
	m_minStep(std::numeric_limits<double>::max()),
	m_currentTime(-1) {
}

Clock& Clock::instance() {
	static Clock inst;
	return inst;
}

double Clock::currentTime() {
	return Clock::instance().m_currentTime;
}

void Clock::addObserver(ClockObserver* obs, double delay) {
	std::cerr << "obs " << obs << " " << delay << "\n";
	Clock& inst = Clock::instance();
	bool found = false;

	// Find the observer and modify it.
	for(ClockObserverItem& item : inst.m_observers) {
		if(item.item == obs) {
			item.delay = delay;
			found = true;
			break;
		}
	}

	// If no item was found, add it.
	if(!found)
		inst.m_observers.emplace_back(obs, delay);

	// Reset minStep
	double minStep = std::numeric_limits<double>::max();
	for(ClockObserverItem& item : inst.m_observers) {
		if(item.delay < minStep)
			minStep = item.delay;
	}
	std::lock_guard<std::mutex> lk(inst.m_stepMtx);
	inst.m_minStep = minStep;
}

void Clock::removeObserver(ClockObserver* obs) {
	Clock& inst = Clock::instance();
	for(auto it = inst.m_observers.begin(); it != inst.m_observers.end(); ++it)
		it = inst.m_observers.erase(it);
}

void Clock::start() {
	Clock& inst = Clock::instance();
	if(!inst.m_running) {
		inst.m_running = true;
		inst.m_thread = std::thread(__clockRun, &(inst.m_observers), &(inst.m_running), &(inst.m_currentTime), &(inst.m_minStep));
	}
}

void Clock::stop() {
	Clock& inst = Clock::instance();
	if(inst.m_running) {
		inst.m_running = false;
		if(inst.m_thread.joinable())
			inst.m_thread.join();
	}
}


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
	return (double) std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000000;
}



Gaussian::Gaussian() : Gaussian(0, 1) {}

Gaussian::Gaussian(double mean, double stddev) {
	setMean(mean);
	setStdDev(stddev);
	m_distribution.param(std::normal_distribution<double>::param_type{m_mean, m_stddev});
}

void Gaussian::setStdDev(double stddev) {
	m_stddev = stddev;
	m_distribution.param(std::normal_distribution<double>::param_type{m_mean, m_stddev});
}

void Gaussian::setMean(double mean) {
	m_mean = mean;
	m_distribution.param(std::normal_distribution<double>::param_type{m_mean, m_stddev});
}

double Gaussian::next() {
	return m_distribution(m_gen);
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

double uav::util::angle(double x, double y) {
	return std::atan2(x, y);
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
