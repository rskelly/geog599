/*
 * gimbal.hpp
 *
 *  Created on: May 18, 2018
 *      Author: rob
 */

#ifndef INCLUDE_GIMBAL_HPP_
#define INCLUDE_GIMBAL_HPP_

#include <Eigen/Core>

/**
 * The gimbal is a device upon which another device, such as a rangefinder is mounted.
 * It may have a fixed position, or it may move, either on command or
 * by a regular internal drive sequence. The gimbal's only job
 * is to calculate a position and rotation matrix, given its own
 * position and rotation, and its state.
 */
class Gimbal {
public:

	/**
	 * The dynamic rotation of the object mounted on this gimbal.
	 * This is the static rotation plus the dynamic rotation.
	 *
	 * @return The dynamic rotation of the object mounted on this gimbal.
	 */
	virtual const Eigen::Matrix3d& rotation() const = 0;

	/**
	 * The dynamic position of the object mounted on this gimbal.
	 * This is the static position plus the dynamic rotation.
	 *
	 * @return The dynamic position of the object mounted on this gimbal.
	 */
	virtual const Eigen::Vector3d& position() const = 0;

	/**
	 * Set the static rotation of the gimbal.
	 *
	 * @param mtx The static rotation of the gimbal.
	 */
	virtual void setStaticRotation(const Eigen::Matrix3d& mtx) = 0;

	/**
	 * Get the static rotation of the gimbal.
	 *
	 * @return The static rotation of the gimbal.
	 */
	virtual const Eigen::Matrix3d& staticRotation() const = 0;

	/**
	 * Set the static position of the gimbal.
	 *
	 * @param mtx The static position of the gimbal.
	 */
	virtual void setStaticPosition(const Eigen::Vector3d& mtx) = 0;

	/**
	 * Get the static position of the gimbal.
	 *
	 * @return The static position of the gimbal.
	 */
	virtual const Eigen::Vector3d& staticPosition() const = 0;

	virtual ~Gimbal() {}
};



#endif /* INCLUDE_GIMBAL_HPP_ */
