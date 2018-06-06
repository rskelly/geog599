/*
 * gimbal.hpp
 *
 *  Created on: May 18, 2018
 *      Author: rob
 */

#ifndef INCLUDE_GIMBAL_HPP_
#define INCLUDE_GIMBAL_HPP_

#include <Eigen/Core>

namespace uav {

/**
 * The gimbal is a device upon which another device, such as a rangefinder is mounted.
 * It may have a fixed position, or it may move, either on command or
 * by a regular internal drive sequence. The gimbal's only job
 * is to calculate a position and orientation matrix, given its own
 * position and orientation, and its state.
 *
 * Example: If this is a rangefinder, its notal point is at the dynamic position,
 * and its beam is aligned with the dynamic orientation. The gimbal itself is positioned
 * at the static orientation and aligned with the dynamic position.
 */
class Gimbal {
protected:

	Gimbal() {}

public:

	/**
	 * Set the dynamic orientation of the object mounted on this gimbal.
	 * Euler angles.
	 *
	 * @param orientation The dynamic orientation of the object mounted on this gimbal.
	 */
	virtual void setOrientation(const Eigen::Vector3d& orientation) = 0;

	/**
	 * Return the dynamic orientation of the object mounted on this gimbal.
	 * Euler angles.
	 *
	 * @return The dynamic orientation of the object mounted on this gimbal.
	 */
	virtual const Eigen::Vector3d& orientation() const = 0;

	/**
	 * Set the dynamic position of the object mounted on this gimbal.
	 * Euler angles.
	 *
	 * @param position The dynamic position of the object mounted on this gimbal.
	 */
	virtual void setPosition(const Eigen::Vector3d& position) = 0;

	/**
	 * The dynamic position of the object mounted on this gimbal.
	 *
	 * @return The dynamic position of the object mounted on this gimbal.
	 */
	virtual const Eigen::Vector3d& position() const = 0;

	/**
	 * Set the static orientation of the gimbal. This is the orientation relative
	 * to the platform frame. Euler angles.
	 *
	 * @param mtx The static orientation of the gimbal.
	 */
	virtual void setStaticOrientation(const Eigen::Vector3d& mtx) = 0;

	/**
	 * Get the static orientation of the gimbal. This is the orientation relative
	 * to the platform frame.
	 *
	 * @return The static orientation of the gimbal.
	 */
	virtual const Eigen::Vector3d& staticOrientation() const = 0;

	/**
	 * Set the static position of the gimbal. This is the position relative
	 * to the platform frame.
	 *
	 * @param mtx The static position of the gimbal.
	 */
	virtual void setStaticPosition(const Eigen::Vector3d& mtx) = 0;

	/**
	 * Get the static position of the gimbal. This is the orientation relative
	 * to the platform frame.
	 *
	 * @return The static position of the gimbal.
	 */
	virtual const Eigen::Vector3d& staticPosition() const = 0;

	virtual ~Gimbal() {}
};

} // uav

#endif /* INCLUDE_GIMBAL_HPP_ */
