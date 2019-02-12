/*
 * pipeline.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#include <unistd.h>

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "TrajectoryPlanner.hpp"
#include "HullPointFilter.hpp"
#include "GeomPointFilter.hpp"
#include "PlaneFilter.hpp"
#include "PointSorter.hpp"
#include "sim/LASPointSource.hpp"

using namespace uav;
using namespace uav::sim;

int main(int argc, char** argv) {

	using namespace Eigen;

	std::string ptsFile = "/home/rob/Documents/msc/data/lidar/nrcan_4.las";
	double startx;
	double starty;
	double endx;
	double endy;
	double laserAngle;
	double scanAngle;
	double altitude;

	double planeWidth = 20;
	double maxDist = 0.01;

	// Hypotenuse (angled range).
	double h = altitude / std::sin(laserAngle);
	Matrix3d rot = AngleAxisf(laserAngle, Vector3f::UnitX());
	Vector3d norm = Vector3d(0, 0, 1) * rot;
	Matrix3d dir = norm * AngleAxisf(90.0 * M_PI / 180.0, Vector3f::UnitX());
	Vector3d orig = Vector3d(startx, starty, 0);
	double planeWidth = scanAngle * h;
	Eigen::Hyperplane plane(norm, orig);
	Eigen::ParametrizedLine line(orig, dir);

	LASPointSource<Pt> tps(ptsFile);
	PlaneFilter<Pt> ppf(planeWidth, maxDist);
	ppf.setOctree(&(tps.octree()));
	ppf.setPlane(&plane);
	ppf.setLine(&line);

	HullPointFilter<Pt> hpf(10.0);
	GeomPointFilter<Pt> gpf;
	gpf.setNextFilter(&hpf);

	TrajectoryPlanner<Pt> tp;
	tp.setPointFilter(&gpf);
	tp.setPointSource(&tps);

	double y = starty;
	double speed = 10.0; // m/s
	int delay = 100;

	tp.start();

	while(true) {

		gpf.setMinY(y);

		orig += // need to move along a vector

		plane = Eigen::Hyperplane(norm, orig);
		line = Eigen::ParametrizedLine(orig, dir);

		ppf.setPlane(&plane);
		ppf.setLine(&line);

		y += speed * ((double) delay / 1000);
		usleep(delay * 1000);
	}

	tp.stop();
}
