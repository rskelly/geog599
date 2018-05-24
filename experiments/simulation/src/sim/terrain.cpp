/*
 * terrain.cpp
 *
 *  Created on: May 23, 2018
 *      Author: rob
 */

#include <vector>

#include <gdal_priv.h>

#include "sim/terrain.hpp"


using namespace uav::sim;

Terrain::Terrain(const std::string& demfile) {

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

double Terrain::sample(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
	return 0;
}

double range(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
	return 0;
}
