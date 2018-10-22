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
	m_tri(new Delaunay2()) {
}

void DelaunaySurface::addPoint(const Eigen::Vector3d& point, double time) {
	Delaunay2::Point pt(point[0], point[1], point[2]);

	std::lock_guard<std::mutex> lk(m_mtx);
	m_tri->insert(pt);

	// Cull any point older than 10s.
	// TODO: This should be a spatial filter, not a temporal one. Works for now.
	size_t t = (size_t) time;
	m_pts[t].push_back(pt);
	auto it = m_pts.begin();
	while(it != m_pts.end()) {
		if(t - it->first > 10) {
			for(Delaunay2::Point& pt0 : it->second)
				m_tri->remove(m_tri->nearest_vertex(pt0));
			it = m_pts.erase(it);
		} else {
			++it;
		}
	}
}

void DelaunaySurface::getVertices(std::vector<double>& vertices) {
	std::lock_guard<std::mutex> lk(m_mtx);
	for(Delaunay2::Finite_faces_iterator it = m_tri->finite_faces_begin();
			it != m_tri->finite_faces_end(); ++it){
		Delaunay2::Triangle t = m_tri->triangle(it);
		for(int i = 0; i < 3; ++i) {
			Point3 v = t.vertex(i);
			vertices.push_back((double) v.x());
			vertices.push_back((double) v.y());
			vertices.push_back((double) v.z());
		}
	}
}

DelaunaySurface::~DelaunaySurface() {}




AlphaSurface::AlphaSurface() :
	m_alpha(0) {

	m_tri.reset(new Alpha3());
}

void AlphaSurface::setAlpha(double alpha) {
	m_alpha = alpha;
	m_tri->set_alpha(alpha);
}

double AlphaSurface::alpha() const {
	return m_alpha;
}

class _alpha_inserter {
private:
	std::vector<double>* vertices;
public:
	_alpha_inserter(std::vector<double>* vertices) :
		vertices(vertices) {}

	void operator =(const Alpha3::Vertex_handle& pt) {
		vertices->push_back(pt->point().x());
		vertices->push_back(pt->point().y());
		vertices->push_back(pt->point().z());
	}

	_alpha_inserter& operator *() {
		return *this;
	}

	_alpha_inserter operator ++(int x) {
		return *this;
	}

	_alpha_inserter& operator ++() {
		return *this;
	}

};

void AlphaSurface::getVertices(std::vector<double>& vertices) {
	{
		// Generate the new alpha shape. Do it here rather than on each add call.
		std::list<Alpha3::Point> pts;
		for(const auto& it : m_pts) {
			for(const Alpha3::Point pt : it.second)
				pts.emplace_back(pt);
		}
		m_tri->make_alpha_shape(pts.begin(), pts.end());
	}
	m_tri->get_alpha_shape_vertices(_alpha_inserter(&vertices), Alpha3::Classification_type::REGULAR);
}

void AlphaSurface::addPoint(const Eigen::Vector3d& point, double time) {
	{
		// Delete old points.
		// TODO: Use a spatial filter.
		std::list<size_t> rem;
		for(const auto& it : m_pts) {
			if(time - (double) it.first > 10.0)
				rem.push_back(it.first);
		}
		for(size_t t : rem)
			m_pts.erase(t);
	}
	{
		// Add the new point, plus the ghost point, to extend the volume.
		std::vector<Alpha3::Point>& pts = m_pts[(size_t) time];
		pts.emplace_back(point.x(), point.y(), point.z());
		pts.emplace_back(point.x(), point.y(), std::max(0.0, point.z() - 1.0));
	}
}

AlphaSurface::~AlphaSurface() {

}
