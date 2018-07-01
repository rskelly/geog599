/*
 * surface.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: rob
 */

#include <unordered_map>

#include "surface.hpp"

using namespace uav::surface;

DelaunaySurface::DelaunaySurface() :
	m_tri(new Delaunay()) {
}

void DelaunaySurface::addPoint(const Eigen::Vector3d& point, double time) {
	Delaunay::Point pt(point[0], point[1], point[2]);

	std::lock_guard<std::mutex> lk(m_mtx);
	m_tri->insert(pt);

	// Cull any point older than 10s.
	// TODO: This should be a spatial filter, not a temporal one. Works for now.
	size_t t = (size_t) time;
	m_pts[t].push_back(pt);
	auto it = m_pts.begin();
	while(it != m_pts.end()) {
		if(t - it->first > 10) {
			for(Delaunay::Point& pt0 : it->second)
				m_tri->remove(m_tri->nearest_vertex(pt0));
			it = m_pts.erase(it);
		} else {
			++it;
		}
	}
}

void DelaunaySurface::getVertices(std::vector<double>& vertices) {
	std::lock_guard<std::mutex> lk(m_mtx);
	for(Delaunay::Finite_faces_iterator it = m_tri->finite_faces_begin();
			it != m_tri->finite_faces_end(); ++it){
		Delaunay::Triangle t = m_tri->triangle(it);
		for(int i = 0; i < 3; ++i) {
			Point v = t.vertex(i);
			vertices.push_back((double) v.x());
			vertices.push_back((double) v.y());
			vertices.push_back((double) v.z());
		}
	}
}

DelaunaySurface::~DelaunaySurface() {}

