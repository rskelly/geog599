/*
 * surface.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: rob
 */

#include <unordered_map>

#include "surface.hpp"

using namespace uav::surface;

std::unordered_map<size_t, std::vector<Delaunay::Point> > __pts;
size_t __last = 0;

DelaunaySurface::DelaunaySurface() :
	m_tri(new Delaunay()) {
}

void DelaunaySurface::addPoint(const Eigen::Vector3d& point, double time) {
	Delaunay::Point pt(point[0], point[1], point[2]);
	m_tri->insert(pt);

	// Cull the oldest 20s of points from the surface triangulation.
	// TODO: This should be a spatial filter, not a temporal one. Works for now.
	size_t t = (size_t) time;
	__pts[t].push_back(pt);
	if(__last == 0) {
		__last = t;
	} else if(t - __last > 20) {
		size_t t0 = __last;
		for(; t0 < t - 10; ++t0) {
			for(Delaunay::Point& pt0 : __pts[t0])
				m_tri->remove(m_tri->nearest_vertex(pt0));
			__pts.erase(t0);
		}
		__last = t0;
	}
}

void DelaunaySurface::getVertices(std::vector<double>& vertices) {
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

