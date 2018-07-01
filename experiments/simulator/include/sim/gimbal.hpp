/*
 * gimbal.hpp
 *
 *  Created on: May 25, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_GIMBAL_HPP_
#define INCLUDE_SIM_GIMBAL_HPP_

#include <thread>

#include <Eigen/Core>

#include "uav.hpp"

#include "../gimbal.hpp"

namespace uav {
namespace sim {

/**
 * The SinGimbal class carries a rangefinder, sweeping
 * back and forth to form a sin wave pattern as the vehicle moves
 * forward.
 */
class SinGimbal : public uav::Gimbal {
private:
	Eigen::Vector3d m_orientation;
	Eigen::Vector3d m_position;
	Eigen::Vector3d m_staticOrientation;
	Eigen::Vector3d m_staticPosition;

	std::thread m_thread;					///!< The update thread.
	bool m_running;

	double m_sweepAngle;
	double m_sweepFrequency;

public:

	/**
	 * Construct a SinGimbal instance, optionally with
	 * a sweep angle and frequency.
	 *
	 * @param sweepAngle The total angle of sweep in radians. Default pi / 2.
	 * @param sweepFrequency The freequency of sweep, as the number of full cycles per second.
	 */
	SinGimbal(double sweepAngle = PI / 2, double sweepFrequency = 1);

	void setSweepAngle(double sweepAngle);

	void setSweepFrequency(double sweepFrequency);

	void start();

	void stop();

	void setOrientation(const Eigen::Vector3d& orientation);

	const Eigen::Vector3d& orientation() const;

	void setPosition(const Eigen::Vector3d& position);

	const Eigen::Vector3d& position() const;

	void setStaticOrientation(const Eigen::Vector3d& mtx);

	const Eigen::Vector3d& staticOrientation() const;

	void setStaticPosition(const Eigen::Vector3d& mtx);

	const Eigen::Vector3d& staticPosition() const;

	void tick(double tick);

	~SinGimbal();
};


/**
 * The NetGimbal class carries a rangefinder, sweeping
 * back and forth, driven by an angle received through
 * a network socket from the flight controller computer.
 */
class NetGimbal : public uav::Gimbal {
private:
	Eigen::Vector3d m_orientation;
	Eigen::Vector3d m_position;
	Eigen::Vector3d m_staticOrientation;
	Eigen::Vector3d m_staticPosition;
	std::string m_addr;

	std::thread m_thread;					///!< The update thread.
	bool m_running;

public:

	/**
	 * Construct a NetGimbal instance, with an IP to connect to
	 * the angle server.
	 *
	 * @param addr The server address.
	 */
	NetGimbal(const std::string& addr);

	void start();

	void stop();

	void setOrientation(const Eigen::Vector3d& orientation);

	const Eigen::Vector3d& orientation() const;

	void setPosition(const Eigen::Vector3d& position);

	const Eigen::Vector3d& position() const;

	void setStaticOrientation(const Eigen::Vector3d& mtx);

	const Eigen::Vector3d& staticOrientation() const;

	void setStaticPosition(const Eigen::Vector3d& mtx);

	const Eigen::Vector3d& staticPosition() const;

	void tick(double tick);

	~NetGimbal();
};


} // sim
} // uav



#endif /* INCLUDE_SIM_GIMBAL_HPP_ */
