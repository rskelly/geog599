/*
 * terrain.cpp
 *
 *  Created on: May 23, 2018
 *      Author: rob
 */

#include <vector>

#include <gdal_priv.h>

#include "sim/terrain.hpp"
#include "sim/geometry.hpp"

using namespace uav::sim;

Terrain::Terrain() :
	m_width(0), m_height(0),
	m_minz(0), m_maxz(0) {}

Terrain::Terrain(const std::string& demfile) {
	load(demfile);
}

void Terrain::load(const std::string& demfile) {
	GDALAllRegister();
	GDALDataset* ds = (GDALDataset*) GDALOpen(demfile.c_str(), GA_ReadOnly);
	int cols = ds->GetRasterXSize();
	int rows = ds->GetRasterYSize();
	ds->GetGeoTransform(m_trans);
	m_width = std::abs(m_trans[1]) * ds->GetRasterXSize();
	m_height = std::abs(m_trans[5]) * ds->GetRasterYSize();
	GDALRasterBand* band = ds->GetRasterBand(2);
	double nodata = band->GetNoDataValue();

	std::vector<float> data(cols * rows);
	if(CE_None != band->RasterIO(GF_Read, 0, 0, cols, rows, data.data(), cols, rows, GDT_Float32, 0, 0, 0))
		throw std::runtime_error("Failed to load raster.");

	/*
	double minz = 99999;
	double maxz = -99999;
	for(int r = 0; r < rows; ++r) {
		for(int c = 0; c < cols; ++c) {
			double z = data[r * cols + c];
			if(z < minz) minz = z;
			if(z > maxz) maxz = z;
		}
	}

	std::vector<Point> pts;
	size_t i = 0;
	for(int r = 0; r < rows; ++r) {
		for(int c = 0; c < cols; ++c) {
			double x = (double) c / cols;
			double y = (double) r / rows;
			double z = (data[r * cols + c] - minz) / (maxz - minz);
			if(z != nodata)
				pts.emplace_back(x, y, z);
			++i;
		}
	}
	*/

	std::vector<Point> pts;
	size_t i = 0;
	m_minz = 99999;
	m_maxz = -99999;
	for(int r = 0; r < rows; ++r) {
		for(int c = 0; c < cols; ++c) {
			double x = m_trans[0] + c * m_trans[1];
			double y = m_trans[3] + r * m_trans[5];
			double z = data[r * cols + c];
			if(z != nodata) {
				pts.emplace_back(x, y, z);
				if(z < m_minz) m_minz = z;
				if(z > m_maxz) m_maxz = z;
			}
			++i;
		}
	}

	m_tri.reset(new Delaunay(pts.begin(), pts.end()));
}

void Terrain::getVertices(std::vector<double>& vertices) {
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

double Terrain::sample(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
	Point_3 pt(origin[0], origin[1], origin[2]);
	Vector_3 dir(direction[0], direction[1], direction[2]);
	Ray_3 line(pt, dir);
	for(Delaunay::Finite_faces_iterator it = m_tri->finite_faces_begin();
			it != m_tri->finite_faces_end(); ++it){
		Delaunay::Triangle t = m_tri->triangle(it);
		if(CGAL::do_intersect(line, t)) {
			const CGAL::Object result = CGAL::intersection(line, t);
			const Point_3* p = CGAL::object_cast<Point_3>(&result);
			if(p)
				return p->z();
		}
	}
	return std::numeric_limits<double>::quiet_NaN();
}

const double* Terrain::transform() const {
	return m_trans;
}

double Terrain::width() const {
	return m_width;
}

double Terrain::height() const {
	return m_height;
}

double Terrain::minz() const {
	return m_minz;
}

double Terrain::maxz() const {
	return m_maxz;
}

inline double __dist3(const Point_3& a, const Point_3& b) {
	return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2) + std::pow(a.z() - b.z(), 2));
}

double Terrain::range(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
	Point_3 pt(origin[0], origin[1], origin[2]);
	Vector_3 dir(direction[0], direction[1], direction[2]);
	Ray_3 line(pt, dir);
	//std::cerr << "ray: " << line << "\n";
	for(Delaunay::Finite_faces_iterator it = m_tri->finite_faces_begin();
			it != m_tri->finite_faces_end(); ++it){
		Delaunay::Triangle t = m_tri->triangle(it);
		if(CGAL::do_intersect(line, t)) {
			const CGAL::Object result = CGAL::intersection(line, t);
			const Point_3* p = CGAL::object_cast<Point_3>(&result);
			if(p)
				return __dist3(*p, pt);
		}
	}
	return std::numeric_limits<double>::quiet_NaN();
}
