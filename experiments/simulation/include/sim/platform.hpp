/*
 * platform.hpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_PLATFORM_HPP_
#define INCLUDE_SIM_PLATFORM_HPP_

#include "../platform.hpp"

/**
 * A platform simulation. The platform is roughly autonomous, so everything
 * about its state originates here, other than the instruction to update.
 */
class SimPlatform : public Platform {
private:
	Eigen::Matrix3d m_orientation;
	Eigen::Vector3d m_position;

	double m_forwardVelocity;

public:

	SimPlatform();

	/**
	 * Update the state of the platform. This is the "tick" that
	 * advances the platforms state, meaning its position, attitude, etc.
	 * according to its flight plan.
	 *
	 * @param time A double, representing the time in seconds.
	 */
	void update(double time);


	const Eigen::Matrix3d& orientation() const;

	const Eigen::Vector3d& position() const;

	~SimPlatform();

};



#endif /* INCLUDE_SIM_PLATFORM_HPP_ */
