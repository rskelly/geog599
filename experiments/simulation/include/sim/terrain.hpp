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

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel         K;
typedef CGAL::Triangulation_vertex_base_with_info_3<size_t, K>    	Vb;
typedef CGAL::Triangulation_data_structure_3<Vb>                    Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds, CGAL::Fast_location> Delaunay;
typedef Delaunay::Point                                             Point;

namespace uav {
namespace sim {

/**
 * Provides a means of sampling a terrain, which is loaded from a raster file.
 *
 */
class Terrain {
private:
	std::unique_ptr<Delaunay> m_tri;

public:

	/**
	 * Create a terrain from the given raster.
	 *
	 * @param demfile A terrain raster.
	 */
	Terrain(const std::string& demfile);

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
	 * Compute the length of the vector starting at origin and
	 * coincident with direction.
	 *
	 * @param origin The origin of the ray.
	 * @param direction The direction of the ray.
	 * @return The elevation of the terrain where it is intersected by the ray.
	 */
	double range(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction);
};

} // sim
} // uav

#endif /* INCLUDE_SIM_TERRAIN_HPP_ */
