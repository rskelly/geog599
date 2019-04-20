/*
 * ProfilePointSource.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_SIM_PROFILEPOINTSOURCE_HPP_
#define INCLUDE_SIM_PROFILEPOINTSOURCE_HPP_

#include <string>
#include <fstream>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ds/Octree.hpp"

namespace uav {
namespace geog599 {

/**
 * Reads points from a CSV file into an Octree.
 * Provides the ability to stream points based on the intersection of a
 * hyperplane and the point cloud. Moving the origin of the plane advances it through
 * the cloud.
 */
template <class P>
class ProfilePointSource {
private:
	std::string m_filename;			///<! The filename of the txt file.
	uav::ds::Octree<P> m_tree;		///<! An Octree to store the points.
	bool m_loaded;					///<! True if a text file is currently loaded.
	Eigen::Vector3d m_planeNormal;	///<! The normal of the hyperplane.
	Eigen::Vector3d m_planeOrigin;	///<! The origin of the hyperplane.
	double m_maxDist;				///<! The maximum distance between a returned point and the hyperplane.

	std::unordered_set<size_t> m_seen; 	///<! Map to prevent duplicates. TODO: Hack.

public:

	/**
	 * Construct the ProfilePointSource using the given LAS file.
	 *
	 * \param filename A CSV file.
	 */
	ProfilePointSource(const std::string& filename) :
		ProfilePointSource() {
		load(filename);
	}

	/**
	 * Construct an empty ProfilePointSource.
	 */
	ProfilePointSource() :
		m_loaded(false),
		m_maxDist(1) {}

	/**
	 * Set the maximum distance from a point to the hyperplane.
	 *
	 * \param d The maximum distance.
	 */
	void setMaxDist(double d) {
		m_maxDist = d;
	}

	/**
	 * Set the origin of the hyperplane.
	 *
	 * \param origin The origin of the hyperplane.
	 */
	void setOrigin(const Eigen::Vector3d& origin) {
		m_planeOrigin = origin;
	}

	/**
	 * Set the normal of the hyperplane.
	 *
	 * \param normal The normal of the hyperplane.
	 */
	void setNormal(const Eigen::Vector3d& normal) {
		m_planeNormal = normal;
	}

	/**
	 * Load a point source file.
	 *
	 * \param filename A CSV point file.
	 */
	void load(const std::string& filename) {
		m_seen.clear();
		m_loaded = false;
		m_filename = filename;
		m_tree.clear();
		{
			double min = std::numeric_limits<double>::lowest();
			double max = std::numeric_limits<double>::max();
			double bounds[] = {max, min, max, min, max, min};
			std::ifstream input(filename);
			std::string buf;
			std::getline(input, buf);
			while(std::getline(input, buf)) {
				std::stringstream ss(buf);
				std::getline(ss, buf, ',');
				double x = atof(buf.c_str());
				std::getline(ss, buf, ',');
				double y = atof(buf.c_str());
				std::getline(ss, buf);
				double z = atof(buf.c_str());
				if(x < bounds[0]) bounds[0] = x;
				if(x > bounds[1]) bounds[1] = x;
				if(y < bounds[2]) bounds[2] = y;
				if(y > bounds[3]) bounds[3] = y;
				if(z < bounds[4]) bounds[4] = z;
				if(z > bounds[5]) bounds[5] = z;
			}
			m_tree.setBounds(bounds);
		}
		{
			std::ifstream input(filename);
			std::string buf;
			std::getline(input, buf);
			size_t i = 0;
			size_t skip = 1;
			while(std::getline(input, buf)) {
				if(++i % skip != 0)
					continue;
				std::stringstream ss(buf);
				std::getline(ss, buf, ',');
				double x = atof(buf.c_str());
				std::getline(ss, buf, ',');
				double y = atof(buf.c_str());
				std::getline(ss, buf);
				double z = atof(buf.c_str());
				m_tree.add(P(x, y, z));
			}
		}
		m_loaded = true;
	}

	/**
	 * Reload the CSV file.
	 */
	void reset() {
		if(m_loaded)
			load(m_filename);
	}

	/**
	 * Compute the bounds of the point cloud and write to the given six-element
	 * array (xmin, xmax, ymin, ymax, zmin, zmax).
	 *
	 * \param bounds A six-element list to contain the point cloud bounds.
	 */
	void computeBounds(double* bounds) {
		if(!m_loaded)
			throw std::runtime_error("Point source is not loaded. Call load first.");
		m_tree.getBounds(bounds);
	}

	/**
	 * Return a reference to the Octree used by this class.
	 *
	 * \param The Octree.
	 */
	const uav::ds::Octree<P>& octree() const {
		return m_tree;
	}

	int getPoints(std::list<P>& pts) {
		pts.clear();
		Eigen::Hyperplane<double, 3> plane(m_planeNormal, m_planeOrigin);
		m_tree.planeSearch(plane, m_maxDist, pts);
		pts.remove_if([=](const P& p) { return p.y() < m_planeOrigin(1) || (m_seen.find(p.time()) != m_seen.end()); });
		for(P& p : pts)
			m_seen.insert(p.time());
		//std::cout << pts.size() << "\n";
		return pts.size();
	}

	/**
	 * Destroy the ProfilePointSource.
	 */
	~ProfilePointSource() {
	}

};

} // geog599
} // uav

#endif /* INCLUDE_SIM_PROFILEPOINTSOURCE_HPP_ */
