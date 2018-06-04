/*
 * terrain.hpp
 *
 *  Created on: May 22, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_TERRAIN_HPP_
#define INCLUDE_SIM_TERRAIN_HPP_

#include <string>
#include <memory>

#include <Eigen/Core>

#include "sim/geometry.hpp"

namespace uav {
namespace sim {

/**
 * Provides a means of sampling a terrain, which is loaded from a raster file.
 *
 */
class Terrain {
private:
	std::unique_ptr<Delaunay> m_tri;
	double m_trans[6];
	double m_width;
	double m_height;
	double m_minz;
	double m_maxz;

public:

	Terrain();

	/**
	 * Create a terrain from the given raster.
	 *
	 * @param demfile A terrain raster.
	 */
	Terrain(const std::string& demfile);

	/**
	 * Load a terrain DEM.
	 *
	 * @param demfile A terrain raster.
	 */
	void load(const std::string& demfile);

	/**
	 * Sample the terrain using the ray originating at origin
	 * and oriented along direction.
	 *
	 * @param origin The origin of the ray.
	 * @param direction The direction of the ray.
	 * @return The elevation of the terrain where it is intersected by the ray.
	 */
	double sample(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction);

	/**
	 * Add the 3D vertices of the triangulation to the vector.
	 *
	 * @param vertices A vector to contain the vertices in x, y, z order.
	 */
	void getVertices(std::vector<double>& vertices);

	/**
	 * Compute the length of the vector starting at origin and
	 * coincident with direction.
	 *
	 * @param origin The origin of the ray.
	 * @param direction The direction of the ray.
	 * @return The elevation of the terrain where it is intersected by the ray.
	 */
	double range(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction);

	/**
	 * Returns the affine transform of the terrain image (GDAL).
	 *
	 * @return The affine transform of the terrain image (GDAL).
	 */
	const double* transform() const;

	double width() const;

	double height() const;

	double minz() const;
	double maxz() const;

};

} // sim
} // uav

#endif /* INCLUDE_SIM_TERRAIN_HPP_ */
