/*
 * terrain.cpp
 *
 *  Created on: May 23, 2018
 *      Author: rob
 */

#include <vector>

#include <gdal_priv.h>

#include <CGAL/Line_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Object.h>

#include "sim/terrain.hpp"

using namespace uav::sim;

Terrain::Terrain() {}

Terrain::Terrain(const std::string& demfile) {
	load(demfile);
}

void Terrain::load(const std::string& demfile) {
	GDALAllRegister();
	GDALDataset* ds = (GDALDataset*) GDALOpen(demfile.c_str(), GA_ReadOnly);
	int cols = ds->GetRasterXSize();
	int rows = ds->GetRasterYSize();
	double trans[6];
	ds->GetGeoTransform(trans);
	GDALRasterBand* band = ds->GetRasterBand(1);
	double nodata = band->GetNoDataValue();

	std::vector<float> data(cols * rows);
	if(CE_None != band->RasterIO(GF_Read, 0, 0, cols, rows, data.data(), cols, rows, GDT_Float32, 0, 0, 0))
		throw std::runtime_error("Failed to load raster.");

	cols = 100;

	std::vector<std::pair<Point, size_t> > pts;
	size_t i = 0;
	for(int r = 0; r < rows; ++r) {
		for(int c = 0; c < cols; ++c) {
			double x = ((double) c / cols) * 2.0 - 1.0;
			double y = (double) r / rows;
			double z = data[r * cols + c];
			if(z == nodata)
				continue;
			pts.emplace_back(Point(x, y, z), i);
			++i;
		}
	}

	m_tri.reset(new Delaunay(pts.begin(), pts.end()));

}

void Terrain::getVertices(std::vector<double>& vertices) {
	Delaunay::Finite_facets_iterator it;
	for(it = m_tri->finite_facets_begin(); it != m_tri->finite_facets_end(); ++it){
		Delaunay::Triangle t = m_tri->triangle(*it);
		Delaunay::Vertex v = t.vertex(0);
		Delaunay::Vertex::Point p = v.point();
		vertices.push_back((double) p.x());
		vertices.push_back((double) p.y());
		vertices.push_back((double) p.z());
	}
}

typedef CGAL::Point_3<K> Point_3;
typedef CGAL::Vector_3<K> Vector_3;
typedef CGAL::Ray_3<K> Ray_3;

double Terrain::sample(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
	Point_3 pt(origin[0], origin[1], origin[2]);
	Vector_3 vec(direction[0], direction[1], direction[2]);
	Ray_3 line(pt, vec);
	Delaunay::Finite_facets_iterator it;
	for(it = m_tri->finite_facets_begin(); it != m_tri->finite_facets_end(); ++it){
		Delaunay::Triangle t = m_tri->triangle(*it);
		if(CGAL::do_intersect(line, t)) {
			const CGAL::Object result = CGAL::intersection(line, t);
			const Point_3* p = CGAL::object_cast<Point_3>(&result);
			if(p)
				return p->z();
		}
	}
	return std::numeric_limits<double>::quiet_NaN();
}

inline double __dist3(const Point_3& a, const Point_3& b) {
	return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2) + std::pow(a.z() - b.z(), 2));
}

double Terrain::range(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
	Point_3 pt(origin[0], origin[1], origin[2]);
	Vector_3 vec(direction[0], direction[1], direction[2]);
	Ray_3 line(pt, vec);
	Delaunay::Finite_facets_iterator it;
	for(it = m_tri->finite_facets_begin(); it != m_tri->finite_facets_end(); ++it){
		Delaunay::Triangle t = m_tri->triangle(*it);
		if(CGAL::do_intersect(line, t)) {
			const CGAL::Object result = CGAL::intersection(line, t);
			const Point_3* p = CGAL::object_cast<Point_3>(&result);
			if(p)
				return __dist3(*p, pt);
		}
	}
	return std::numeric_limits<double>::quiet_NaN();
}
