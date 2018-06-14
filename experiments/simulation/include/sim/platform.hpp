/*
 * platform.hpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_PLATFORM_HPP_
#define INCLUDE_SIM_PLATFORM_HPP_

#include <thread>

#include "util.hpp"
#include "surface.hpp"
#include "../platform.hpp"
#include "../rangefinder.hpp"

namespace uav {
namespace sim {


class PlatformControlInput : public uav::PlatformControlInput {
private:
	double m_altitude;
public:
	PlatformControlInput() :
		m_altitude(std::numeric_limits<double>::quiet_NaN()) {
	}

	/**
	 * Set the altitude.
	 *
	 * @param altitude The altitude.
	 */
	void setAltitude(double altitude) {
		m_altitude = altitude;
	}

	/**
	 * Get the altitude.
	 *
	 * @return The altitude.
	 */
	double altitude() const {
		return m_altitude;
	}
	/**
	 * Merge the input into the current input.
	 *
	 * @param input A PlatformControlInput.
	 */
	void operator<<(const uav::PlatformControlInput& input) {
		setAltitude(input.altitude());
	}

	/**
	 * Reset to the initial state.
	 */
	void reset() {
		m_altitude = std::numeric_limits<double>::quiet_NaN();
	}
};


class PlatformState : public uav::PlatformState {
private:
	Eigen::Vector3d m_orientation;   ///<! The orientation matrix. Euler angles.
	Eigen::Vector3d m_position;      ///<! The position. Coordinates in whatever Cartesian space.
	Eigen::Vector3d m_linearVelocity;
	Eigen::Vector3d m_angularVelocity;
	double m_batteryLevel;
	double m_mass;
	double m_altitude;
	double m_altitudeTime;

public:

	PlatformState();

	void setPosition(const Eigen::Vector3d& position);

	void setOrientation(const Eigen::Vector3d& orientation);

	void setBatteryLevel(double level);

	void setMass(double mass);

	void setAltitude(double altitude);

	void setAltitudeTime(double time);

	void setLinearVelocity(const Eigen::Vector3d& linearVelocity);

	void setAngularVelocity(const Eigen::Vector3d& angularVelocity);

	const Eigen::Vector3d& position() const;

	const Eigen::Vector3d& orientation() const;

	const Eigen::Vector3d& linearVelocity() const;

	const Eigen::Vector3d& angularVelocity() const;

	double batteryLevel() const;

	double mass() const;

	double altitude() const;

	double altitudeTime() const;

};

/**
 * Represents the state of the rangefinder.
 */
class RangefinderState : public uav::RangefinderState {
private:
	Eigen::Vector3d m_position;
	Eigen::Vector3d m_direction;

public:

	void setLaserPosition(const Eigen::Vector3d& position);

	void setLaserDirection(const Eigen::Vector3d& direction);

	const Eigen::Vector3d& laserPosition() const;

	const Eigen::Vector3d& laserDirection() const;
};

/**
 * A platform simulation. The platform is roughly autonomous, so everything
 * about its state originates here, other than the instruction to update.
 */
class Platform : public uav::Platform,	public uav::RangefinderObserver {
private:
	uav::sim::PlatformState m_platformState;
	uav::sim::RangefinderState m_rangefinderState;
	uav::sim::RangefinderState m_nadirRangefinderState;
	uav::sim::PlatformControlInput m_controlInput;

	uav::util::Poisson m_posPoisson; ///<! Generator for Poisson values.
	uav::util::Poisson m_rotPoisson; ///<! Generator for Poisson values.

	uav::Gimbal* m_gimbal;
	uav::Rangefinder* m_rangefinder;
	uav::Rangefinder* m_nadirRangefinder;
	uav::surface::Surface* m_surface;

public:

	Platform();

	void start();

	void stop();

	void rangeUpdate(uav::Rangefinder* rangefinder, uav::Range* range);

	/**
	 * Update the state of the platform. This is the "tick" that
	 * advances the platforms state, meaning its position, attitude, etc.
	 * according to its flight plan. Presumably driven by a Controller.
	 *
	 * @param time A double, representing the time in seconds.
	 */
	void update(double time);

	/**
	 * Set the initial platform state for simulation.
	 *
	 * @param state The initial platform state.
	 */
	void setInitialPlatformState(const uav::sim::PlatformState& state);

	const uav::RangefinderState& nadirRangefinderState() const;

	const uav::RangefinderState& rangefinderState() const;

	void setGimbal(uav::Gimbal* gimbal);

	uav::Gimbal* gimbal() const;

	void setRangefinder(uav::Rangefinder* rangefinder);

	uav::Rangefinder* rangefinder() const;

	void setNadirRangefinder(uav::Rangefinder* rangefinder);

	uav::Rangefinder* nadirRangefinder() const;

	void setSurface(uav::surface::Surface* surface);

	uav::surface::Surface* surface() const;

	const uav::PlatformState& platformState() const;

	void setControlInput(const uav::PlatformControlInput& input);

	~Platform();

};

} // sim
} // uav

#endif /* INCLUDE_SIM_PLATFORM_HPP_ */
