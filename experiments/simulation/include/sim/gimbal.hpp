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

#include "../gimbal.hpp"

namespace uav {
namespace sim {

class Gimbal : public uav::Gimbal {
private:
	Eigen::Vector3d m_orientation;
	Eigen::Vector3d m_position;
	Eigen::Vector3d m_staticOrientation;
	Eigen::Vector3d m_staticPosition;

	std::thread* m_thread;

public:

	Gimbal();

	const Eigen::Vector3d& orientation() const;

	const Eigen::Vector3d& position() const;

	void setStaticOrientation(const Eigen::Vector3d& mtx);

	const Eigen::Vector3d& staticOrientation() const;

	void setStaticPosition(const Eigen::Vector3d& mtx);

	const Eigen::Vector3d& staticPosition() const;

	~Gimbal();
};

} // sim
} // uav



#endif /* INCLUDE_SIM_GIMBAL_HPP_ */
