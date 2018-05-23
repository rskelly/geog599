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

	GDALDataset* ds = (GDALDataset*) GDALOpen(demfile.c_str(), GA_ReadOnly);
	int cols = ds->GetRasterXSize();
	int rows = ds->GetRasterYSize();
	double trans[6];
	ds->GetGeoTransform(trans);
	GDALRasterBand* band = ds->GetRasterBand(1);

	std::vector<float> data(cols * rows);
	band->RasterIO(GF_Read, 0, 0, cols, rows, data.data(), cols, rows, GDT_Float32, 0, 0, 0);

	std::vector<std::pair<Point, size_t> > pts;
	size_t i = 0;
	for(int r = 0; r < rows; ++r) {
		for(int c = 0; c < cols; ++c) {
			double x = trans[0] + c * trans[1];
			double y = trans[3] + r * trans[5];
			double z = data[r * cols + c];
			pts.emplace_back(Point(x, y, z), i);
			++i;
		}
	}

	m_tri.reset(new Delaunay(pts.begin(), pts.end()));

}

double Terrain::sample(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
	return 0;
}

double range(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
	return 0;
}
