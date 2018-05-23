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
	Eigen::Matrix3d m_rotation;
	Eigen::Vector3d m_position;

	uav::util::Poisson m_posPoisson;
	double m_forwardVelocity;
	double m_lastTime;

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


	const Eigen::Matrix3d& rotation() const;

	const Eigen::Vector3d& position() const;

	~Platform();

};

} // sim
} // uav

#endif /* INCLUDE_SIM_PLATFORM_HPP_ */
