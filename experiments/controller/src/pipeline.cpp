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

#include <QtWidgets/QApplication>

#include "TrajectoryPlanner.hpp"
#include "HullPointFilter.hpp"
#include "GeomPointFilter.hpp"
#include "PlaneFilter.hpp"
#include "PointSorter.hpp"
#include "sim/LASPointSource.hpp"

#include "ui/profile.hpp"
#include "ui/drawconfig.hpp"

using namespace uav;
using namespace uav::sim;
using namespace Eigen;

double _rad(double deg) {
	return deg * M_PI / 180.0;
}

Matrix3d rotationMatrix(double rotX, double rotY, double rotZ) {
	double sx = std::sin(rotX), cx = std::cos(rotX);
	double sy = std::sin(rotY), cy = std::cos(rotY);
	double sz = std::sin(rotZ), cz = std::cos(rotZ);
	Matrix3d rotx; rotx << 1, 0, 0, 0, cx, -sx, 0, sx, cx;
	Matrix3d roty; roty << cy, 0, sy, 0, 1, 0, -sy, 0, cy;
	Matrix3d rotz; rotz << cz, -sz, 0, sz, cz, 0, 0, 0, 1;
	return rotz * roty * rotx;
}

void run() {

	/*
	Vector3d pt(0, 1, 0);

	Vector3d trans(1, 0, 0);

	double rotX = M_PI / 4;
	double rotY = M_PI / 4;
	double rotZ = 0;

	Matrix3d rot = rotationMatrix(rotX, rotY, rotZ);

	pt += trans;

	Vector3d out = rot * pt;

	std::cerr << rot << "\n\n";
	std::cerr << pt << "\n\n";
	std::cerr << out << "\n";
	*/

	std::string ptsFile = "/home/rob/Documents/msc/data/lidar/nrcan_4.las";
	double startx = 620154.92241;
	double starty = 5613102.36059;
	double endx = 620195.30108;
	double endy = 5613635.35912;
	double laserAngle = _rad(15);
	double scanAngle = _rad(15);
	double altitude = 310;
	double offset = 10; // vertical

	altitude += offset;

	double maxDist = 0.01;

	// Hypotenuse (angled range).
	double h = altitude / std::sin(laserAngle);

	Vector3d orig(startx, starty, altitude);
	Vector3d start(startx, starty, altitude);
	Vector3d end(endx, endy, altitude);

	Vector3d direction = (end - start).normalized();
	Vector3d xaxis(direction[1], -direction[0], direction[2]);

	Matrix3d laserRot = AngleAxisd(-laserAngle, xaxis).matrix();
	Matrix3d planeRot = AngleAxisd(-laserAngle + M_PI / 2.0, xaxis).matrix();

	Vector3d laserDir = laserRot * direction;
	Vector3d planeNorm = planeRot * laserDir;

	std::cout << "Direction: " << direction[0] << ", " << direction[1] << ", " << direction[2] << "\n";
	std::cout << "Laser vector: " << laserDir[0] << ", " << laserDir[1] << ", " << laserDir[2] << "\n";
	std::cout << "Plane normal: " << planeNorm[0] << ", " << planeNorm[1] << ", " << planeNorm[2] << "\n";

	double planeWidth = scanAngle * h;
	Eigen::Hyperplane<double, 3> plane(planeNorm, orig);
	Eigen::ParametrizedLine<double, 3> line(orig, laserDir);

	LASPointSource<Pt> tps(ptsFile);
	PlaneFilter<Pt> ppf(planeWidth, maxDist);
	ppf.setOctree(&(tps.octree()));
	ppf.setPlane(&plane);
	ppf.setLine(&line);
	tps.setFilter(&ppf);

	HullPointFilter<Pt> hpf(5.0);
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

	Vector3d step(stepx, stepy, 0);

	//tp.start();

	/*
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
	*/

	DrawConfig uav;
	uav.setType(DrawType::Points);
	uav.setLineColor(255, 0, 0);

	DrawConfig alt;
	alt.setType(DrawType::Line);
	alt.setLineColor(0, 100, 0);

	DrawConfig allPts;
	allPts.setType(DrawType::Points);
	allPts.setLineColor(63, 63, 63);

	DrawConfig surf;
	surf.setType(DrawType::Line);
	surf.setLineColor(0, 255, 255);

	DrawConfig spline;
	spline.setType(DrawType::Line);
	spline.setLineColor(255, 255, 255);

	ProfileDialog* pd = ProfileDialog::instance();
	pd->setBounds(0, 250, (end - start).norm(), 350);
	pd->addDrawConfig(&allPts);
	pd->addDrawConfig(&surf);
	pd->addDrawConfig(&spline);
	pd->addDrawConfig(&alt);
	pd->addDrawConfig(&uav);

	uav.data.emplace_back(0, altitude + offset);

	while(true) {

		plane = Eigen::Hyperplane<double, 3>(planeNorm, orig);
		line = Eigen::ParametrizedLine<double, 3>(orig, laserDir);

		// To clip off the points in the past.
		double dy = (orig - start).norm();
		gpf.setMinY(dy - 100.0);
		//std::cout << dy << "\n";

		ppf.setPlane(&plane);
		ppf.setLine(&line);

		tp.compute();

		std::list<Pt> salt;
		tp.splineAltitude(salt, 1);

		uav.data[0].first = dy;
		uav.data[0].second = altitude;

		spline.data.clear();
		for(const Pt& pt : salt)
			spline.data.emplace_back(pt.y(), pt.z());
		allPts.data.clear();
		for(const Pt& pt : tp.allPoints())
			allPts.data.emplace_back(pt.y(), pt.z());
		surf.data.clear();
		for(const Pt& pt : tp.surface())
			surf.data.emplace_back(pt.y(), pt.z());

		if(!tp.getTrajectoryAltitude(dy, altitude)) {
			//std::cerr << "Couldn't get new altitude.";
		} else {
			//std::cout << "Altitude: " << altitude << "\n";

			alt.data.emplace_back(dy, altitude + offset);

			/*
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
			*/

			altitude += offset;
			orig[2] = altitude;
			start[2] = altitude;
			end[2] = altitude;
		}

		pd->draw();

		if(std::abs((end - orig).norm()) < 1)
			break;

		orig += step; // need to move along a vector

		usleep(delay);

	}

	//tp.stop();

	std::cerr << "Done\n";
}

int main(int argc, char** argv) {

	QApplication app(argc, argv);
	QDialog w;
	ProfileDialog p;
	p.setupUi(&w);
	w.show();
	std::thread th(run);
	app.exec();


}
