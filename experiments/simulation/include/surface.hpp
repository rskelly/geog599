/*
 * surface.hpp
 *
 *  Created on: Jun 6, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SURFACE_HPP_
#define INCLUDE_SURFACE_HPP_

#include <memory>

#include <Eigen/Core>

#include "geometry.hpp"

namespace uav {
namespace surface {

/**
 * This is the base for the surface reconstructed from
 * laser points collected by the platform. Ultimately
 * this surface is used to estimate a trajectory.
 */
class Surface {
public:

	/**
	 * Add a point to the reconstructed surface. The time is the number of
	 * seconds since the epoch, when the point was collected.
	 *
	 * @param point A 3-dimensional coordinate.
	 * @param time The time the point was collected in seconds since the epoch.
	 */
	virtual void addPoint(const Eigen::Vector3d& point, double time) = 0;

	virtual ~Surface() {}
};

/**
 * This is a specialization of Surface that uses a Delaunay triangulation
 * to compute the surface from sample points.
 */
class DelaunaySurface : public Surface {
private:
	std::unique_ptr<Delaunay> m_tri;

public:

	DelaunaySurface();

	/**
	 * Populate the vector with the vertices of the triangulation.
	 * These will be ordered by face so that they can be fed directly
	 * into the renderer.
	 *
	 * @param vertices A vector that will contain the coordinates.
	 */
	void getVertices(std::vector<double>& vertices);

	void addPoint(const Eigen::Vector3d& point, double time);

	~DelaunaySurface();

};

} // surface
} // uav


#endif /* INCLUDE_SURFACE_HPP_ */
