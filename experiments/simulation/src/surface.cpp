/*
 * surface.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: rob
 */

#include "surface.hpp"

using namespace uav::surface;

DelaunaySurface::DelaunaySurface() :
	m_tri(new Delaunay()) {
}

void DelaunaySurface::addPoint(const Eigen::Vector3d& point, double time) {
	Delaunay::Point pt(point[0], point[1], point[2]);
	m_tri->insert(pt);
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

