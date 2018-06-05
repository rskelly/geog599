/*
 * platform.hpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_PLATFORM_HPP_
#define INCLUDE_SIM_PLATFORM_HPP_

#include "util.hpp"
#include "../platform.hpp"

namespace uav {
namespace sim {

/**
 * A platform simulation. The platform is roughly autonomous, so everything
 * about its state originates here, other than the instruction to update.
 */
class Platform : public uav::Platform {
private:
	Eigen::Vector3d m_orientation;   ///<! The orientation matrix. Euler angles.
	Eigen::Vector3d m_position;      ///<! The position. Coordinates in whatever Cartesian space.

	Eigen::Vector3d m_laserPosition;    ///<! The computerd position of the laser. Only available after update.
	Eigen::Vector3d m_laserDirection;   ///<! The computerd direction of the laser. Only available after update.

	uav::util::Poisson m_posPoisson; ///<! Generator for Poisson values.
	uav::util::Poisson m_rotPoisson; ///<! Generator for Poisson values.
	double m_forwardVelocity;        ///<! The current forward velocity.
	double m_lastTime;               ///<! The last update time.

	Gimbal* m_gimbal;
	Rangefinder* m_rangefinder;

public:

	Platform();

	/**
	 * Update the state of the platform. This is the "tick" that
	 * advances the platforms state, meaning its position, attitude, etc.
	 * according to its flight plan.
	 *
	 * @param time A double, representing the time in seconds.
	 */
	void update(double time);

	/**
	 * Return the current range or nullptr if none has been sensed.
	 *
	 * @return The current range or nullptr if none has been sensed.
	 */
	Range* range() const;

	/**
	 * Return the caculated laser position. Only available after an update.
	 *
	 * @return The caculated laser position. Only available after an update.
	 */
	const Eigen::Vector3d laserPosition() const;

	/**
	 * Return the caculated laser direction. Only available after an update.
	 *
	 * @return The caculated laser direction. Only available after an update.
	 */
	const Eigen::Vector3d laserDirection() const;

	const uav::Gimbal* gimbal() const;

	const uav::Rangefinder* rangefinder() const;

	const Eigen::Vector3d& orientation() const;

	const Eigen::Vector3d& position() const;

	~Platform();

};

} // sim
} // uav

#endif /* INCLUDE_SIM_PLATFORM_HPP_ */
