/*
 * gimbalminimu9v5.hpp
 *
 *  Created on: Dec 26, 2018
 *      Author: rob
 */

#ifndef INCLUDE_IMPL_GIMBALMINIMU9V5_HPP_
#define INCLUDE_IMPL_GIMBALMINIMU9V5_HPP_

#include "gimbal.hpp"
#include "impl/serialbridge.hpp"

namespace uav {
namespace impl {

class GimbalMinIMU9v5 : public uav::Gimbal, public uav::util::ClockObserver, public uav::impl::SerialBridgeListener {
private:
	Eigen::Vector3d m_orientation;
	Eigen::Vector3d m_position;
	Eigen::Vector3d m_staticOrientation;
	Eigen::Vector3d m_staticPosition;
	uint64_t m_timestamp;

public:

	void serialBridgeUpdate(SerialBridge* bridge);

	/**
	 * Set the dynamic orientation of the object mounted on this gimbal.
	 * Euler angles.
	 *
	 * @param orientation The dynamic orientation of the object mounted on this gimbal.
	 */
	void setOrientation(const Eigen::Vector3d& orientation);

	/**
	 * Return the dynamic orientation of the object mounted on this gimbal.
	 * Euler angles.
	 *
	 * @return The dynamic orientation of the object mounted on this gimbal.
	 */
	const Eigen::Vector3d& orientation() const;

	/**
	 * Set the dynamic position of the object mounted on this gimbal.
	 * Euler angles.
	 *
	 * @param position The dynamic position of the object mounted on this gimbal.
	 */
	void setPosition(const Eigen::Vector3d& position);

	/**
	 * The dynamic position of the object mounted on this gimbal.
	 *
	 * @return The dynamic position of the object mounted on this gimbal.
	 */
	const Eigen::Vector3d& position() const;

	/**
	 * Set the static orientation of the gimbal. This is the orientation relative
	 * to the platform frame. Euler angles.
	 *
	 * @param mtx The static orientation of the gimbal.
	 */
	void setStaticOrientation(const Eigen::Vector3d& mtx);

	/**
	 * Get the static orientation of the gimbal. This is the orientation relative
	 * to the platform frame.
	 *
	 * @return The static orientation of the gimbal.
	 */
	const Eigen::Vector3d& staticOrientation() const;

	/**
	 * Set the static position of the gimbal. This is the position relative
	 * to the platform frame.
	 *
	 * @param mtx The static position of the gimbal.
	 */
	void setStaticPosition(const Eigen::Vector3d& mtx);

	/**
	 * Get the static position of the gimbal. This is the orientation relative
	 * to the platform frame.
	 *
	 * @return The static position of the gimbal.
	 */
	const Eigen::Vector3d& staticPosition() const;

	/**
	 * Start the gimbal's operation.
	 */
	void start();

	/**
	 * Stop the gimbal's operation.
	 */
	void stop();

	void tick(double time = 0);

	~GimbalMinIMU9v5();

};

} // impl
} // uav



#endif /* INCLUDE_IMPL_GIMBALMINIMU9V5_HPP_ */
