/*
 * surface.hpp
 *
 *  Created on: Jun 6, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SURFACE_HPP_
#define INCLUDE_SURFACE_HPP_

#include <memory>
#include <mutex>

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

	/**
	 * Populate the vector with the vertices of the triangulation.
	 * These will be ordered by face so that they can be fed directly
	 * into the renderer.
	 *
	 * @param vertices A vector that will contain the coordinates.
	 */
	virtual void getVertices(std::vector<double>& vertices) = 0;

	virtual ~Surface() {}
};

/**
 * This is a specialization of Surface that uses a 3D Delaunay triangulation
 * to compute the surface from sample points.
 */
class DelaunaySurface : public Surface {
private:
	std::unique_ptr<Delaunay2> m_tri;
	std::unordered_map<size_t, std::vector<Delaunay2::Point> > m_pts;
	std::mutex m_mtx;

public:

	DelaunaySurface();

	void getVertices(std::vector<double>& vertices);

	void addPoint(const Eigen::Vector3d& point, double time);

	~DelaunaySurface();

};


/**
 * The alpha surface uses an Alpha Shape algorithm to reconstruct a surface
 * that is not necessarily complex, depending on the value of the
 * alpha parameter. Otherwise it is similar to (and based on) the 3D
 * Delaunay triangulation.
 */
class AlphaSurface : public Surface {
private:
	std::unique_ptr<Alpha3> m_tri;
	std::unordered_map<size_t, std::vector<Alpha3::Point> > m_pts;
	std::mutex m_mtx;
	double m_alpha;

public:

	AlphaSurface();


	/**
	 * Set the alpha value for this surface.
	 *
	 * @param alpha The alpha value for this surface.
	 */
	void setAlpha(double alpha);

	/**
	 * Return the alpha value.
	 *
	 * @return The alpha value.
	 */
	double alpha() const;

	/**
	 * Populate the vector with the vertices of the triangulation.
	 * These will be ordered by face so that they can be fed directly
	 * into the renderer.
	 *
	 * @param vertices A vector that will contain the coordinates.
	 */
	void getVertices(std::vector<double>& vertices);

	/**
	 * Adds a point to the point set. For each point added, an additional
	 * point will be added with the z-component replaced with a global
	 * z, creating a 3D solid.
	 *
	 * @param point A 3-dimensional coordinate.
	 * @param time The time the point was collected in seconds since the epoch.
	 */
	void addPoint(const Eigen::Vector3d& point, double time);

	~AlphaSurface();

};

} // surface
} // uav


#endif /* INCLUDE_SURFACE_HPP_ */
