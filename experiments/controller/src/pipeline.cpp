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
using namespace Eigen;

double _rad(double deg) {
	return deg * M_PI / 180.0;
}

int main(int argc, char** argv) {

	std::string ptsFile = "/home/rob/Documents/msc/data/lidar/nrcan_4.las";
	double startx = 620154.92241;
	double starty = 5613102.36059;
	double endx = 620195.30108;
	double endy = 5613635.35912;
	double laserAngle = _rad(15);
	double scanAngle = _rad(30);
	double altitude = 310;
	double offset = 10; // vertical

	altitude += offset;

	double maxDist = 0.01;

	// Hypotenuse (angled range).
	double h = altitude / std::sin(laserAngle);

	Matrix3d rot;
	rot = AngleAxisd(-laserAngle, Vector3d::UnitX())
			* AngleAxisd(0, Vector3d::UnitY())
			* AngleAxisd(0, Vector3d::UnitZ());

	Matrix3d right;
	right = AngleAxisd(M_PI / 2.0, Vector3d::UnitX())
			* AngleAxisd(0, Vector3d::UnitY())
			* AngleAxisd(0, Vector3d::UnitZ());

	Vector3d norm = rot * Vector3d(0, 0, 1);
	Vector3d dir = right * norm;
	Vector3d orig(startx, starty, altitude);

	double planeWidth = scanAngle * h;
	Eigen::Hyperplane<double, 3> plane(norm, orig);
	Eigen::ParametrizedLine<double, 3> line(orig, dir);

	LASPointSource<Pt> tps(ptsFile);
	PlaneFilter<Pt> ppf(planeWidth, maxDist);
	ppf.setOctree(&(tps.octree()));
	ppf.setPlane(&plane);
	ppf.setLine(&line);
	tps.setFilter(&ppf);

	HullPointFilter<Pt> hpf(2.0);
	GeomPointFilter<Pt> gpf;
	gpf.setNextFilter(&hpf);

	uav::Pt startPt(startx, starty, altitude);
	TrajectoryPlanner<Pt> tp;
	tp.setSmooth(0.8);
	tp.setWeight(0.1);
	tp.setPointFilter(&gpf);
	tp.setPointSource(&tps);
	tp.setStartPoint(startPt);

	double speed = 10.0; // m/s
	int delay = 1000;	// 1 ms

	double stepx = (endx - startx) / (speed * delay); // 10m/s in milis
	double stepy = (endy - starty) / (speed * delay);

	Vector3d start(startx, starty, altitude);
	Vector3d end(endx, endy, altitude);
	Vector3d step(stepx, stepy, 0);

	//tp.start();

	std::ofstream ostr;
	{
		std::stringstream ss;
		ss << "pos_" << tp.smooth() << "_" << tp.weight() << ".csv";
		ostr.open(ss.str());
	}

	std::ofstream tstr;
	{
		std::stringstream ss;
		ss << "traj_" << tp.smooth() << "_" << tp.weight() << ".csv";
		tstr.open(ss.str());
	}

	while(true) {

		plane = Eigen::Hyperplane<double, 3>(norm, orig);
		line = Eigen::ParametrizedLine<double, 3>(orig, dir);

		// To clip off the points in the past.
		double dy = (orig - start).norm();
		gpf.setMinY(dy - 10.0);
		//std::cout << dy << "\n";

		ppf.setPlane(&plane);
		ppf.setLine(&line);

		tp.compute();

		if(!tp.getTrajectoryAltitude(dy, altitude)) {
			//std::cerr << "Couldn't get new altitude.";
		} else {
			//std::cout << "Altitude: " << altitude << "\n";

			ostr << dy << "," << altitude << "," << (altitude + offset) << "\n";

			std::list<Pt> alt;
			std::list<Pt> vel;
			std::list<Pt> accel;
			if(tp.splineAltitude(alt)) {
				if(tp.splineVelocity(vel)) {
					if(tp.splineAcceleration(accel)) {
						tstr << "y,";
						for(const Pt& p : alt)
							tstr << p.y() << ",";
						tstr << "\nalt,";
						for(const Pt& p : alt)
							tstr << p.z() << ",";
						tstr << "\nacc,";
						for(const Pt& p : accel)
							tstr << p.z() << ",";
						tstr << "\nvel,";
						for(const Pt& p : vel)
							tstr << p.z() << ",";
						tstr << "\n";
					}
				}
			}

			altitude += offset;
			orig[2] = altitude;
			start[2] = altitude;
			end[2] = altitude;
		}

		if(std::abs((end - orig).norm()) < 1)
			break;

		orig += step; // need to move along a vector

		usleep(delay);

	}

	//tp.stop();

	std::cerr << "Done\n";
}
