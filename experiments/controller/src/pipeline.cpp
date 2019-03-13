/*
 * pipeline.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#include <unistd.h>

#include <string>
#include <unordered_map>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <QtWidgets/QApplication>

#include "geog599/TrajectoryPlanner.hpp"
#include "geog599/HullPointFilter.hpp"
#include "geog599/GeomPointFilter.hpp"
#include "geog599/PlaneFilter.hpp"
#include "geog599/PointSorter.hpp"
#include "geog599/LASPointSource.hpp"

#include "ui/profile.hpp"
#include "ui/drawconfig.hpp"

#define MAX_RANGE 1000 // The maximum laser range
#define MAX_DIST 1 // The maximum distance between a point and the search plane.
#define TIME_DELAY 100
#define SPEED 10

using namespace uav::ds;
using namespace uav::geog599;
using namespace uav::geog599::filter;
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

class PipelineConfig {
public:
	std::string filename;
	double startAltitude;
	double offset;
	double smooth;
	double weight;
	double alpha;
	double laserAngle;
	double scanAngle;

	PipelineConfig() :
		PipelineConfig("", 0, 0, 0, 0, 0, 0, 0) {}

	PipelineConfig(const std::string& filename, double startAltitude, double offset,
			double smooth, double weight, double alpha, double laserAngle, double scanAngle) :
		filename(filename), startAltitude(startAltitude), offset(offset), smooth(smooth),
		weight(weight), alpha(alpha), laserAngle(laserAngle), scanAngle(scanAngle) {}

};

void run() {

	// Some pre-prepared configurations.
	std::unordered_map<std::string, PipelineConfig> configs;
	configs.emplace("nrcan_1", PipelineConfig("/home/rob/Documents/msc/data/lidar/nrcan_4.las", 305, 10, 5, 1, 15, 5.7, 5));
	configs.emplace("mt_doug_1", PipelineConfig("/home/rob/Documents/msc/data/lidar/mt_doug_1.las", 80, 10, 5, 1, 15, 5.7, 5));

	const PipelineConfig& config = configs["nrcan_1"];

	std::string ptsFile = config.filename;

	LASPointSource<Pt> tps(ptsFile);
	const Octree<Pt>& tree = tps.octree();
	double startx, starty, endx, endy;

	if(tree.width() > tree.length()) {
		startx = tree.minx() - 100.0; // Start 100m back to give the laser space to work.
		endx = tree.maxx();
		starty = tree.midy();
		endy = tree.midy();
	} else {
		startx = tree.midx();
		endx = tree.midx();
		starty = tree.miny() - 100.0; // Start 100m back to give the laser space to work.
		endy = tree.maxy();
	}

	// Offset from trajectory, and vehicle altitude (start altitude + offset)
	double offset = config.offset;
	double altitude = config.startAltitude;

	// Angle down from horizontal (rad) and scan angle (rad)
	double laserAngle = _rad(config.laserAngle); // 10m altitude @ 100m range = 5.7 deg
	double scanAngle = _rad(config.scanAngle);

	// Maximum distance of a point from the scan plane.
	double maxDist = MAX_DIST;

	// Hypotenuse - this is just the maximum range; assumes that the laser
	// angle is set so that the laser intersects flat ground at max range,
	// given the platform elevation.
	double h = MAX_RANGE;

	// The start/origin and end points. These are determined by the point cloud.
	Vector3d orig(startx, starty, altitude);
	Vector3d start(startx, starty, altitude);
	Vector3d end(endx, endy, altitude);

	// The vehicle direction in the horizontal plane.
	Vector3d direction = (end - start).normalized();
	// The pitch axis for the vehicle and the scanning rangefinder.
	Vector3d xaxis(direction[1], -direction[0], direction[2]);

	// The rotation matrix for the rangefinder.
	Matrix3d laserRot = AngleAxisd(-laserAngle, xaxis).matrix();
	// The rotation matrix for the detection plane, perpendicular to the laser.
	Matrix3d planeRot = AngleAxisd(-laserAngle + M_PI / 2.0, xaxis).matrix();

	// The laser's and plane's normal.
	Vector3d laserDir = laserRot * direction;
	Vector3d planeNorm = planeRot * laserDir;

	std::cout << "Direction: " << direction[0] << ", " << direction[1] << ", " << direction[2] << "\n";
	std::cout << "Laser vector: " << laserDir[0] << ", " << laserDir[1] << ", " << laserDir[2] << "\n";
	std::cout << "Plane normal: " << planeNorm[0] << ", " << planeNorm[1] << ", " << planeNorm[2] << "\n";

	// The width of the detection plane.
	double planeWidth = scanAngle * h;

	// The plane and centreline used to detect points in the octree.
	Eigen::Hyperplane<double, 3> plane(planeNorm, orig);
	Eigen::ParametrizedLine<double, 3> line(orig, laserDir);

	// Object for fintering points using the plane and centreline.
	PlaneFilter<Pt> ppf(planeWidth, maxDist);
	ppf.setOctree(&(tps.octree()));
	ppf.setPlane(&plane);
	ppf.setLine(&line);
	tps.setFilter(&ppf);

	// The concave hull point filter.
	HullPointFilter<Pt> hpf(config.alpha);
	GeomPointFilter<Pt> gpf;
	gpf.setNextFilter(&hpf);

	// The trajectory planner manages the point cloud source and filters to
	// compute the trajectory.
	Pt startPt(startx, starty, altitude);
	TrajectoryPlanner<Pt> tp;
	tp.setSmooth(config.smooth);
	tp.setWeight(config.weight);
	tp.setPointFilter(&gpf);
	tp.setPointSource(&tps);
	tp.setStartPoint(startPt);

	double speed = SPEED;
	int delay = TIME_DELAY;

	double stepx = (endx - startx) / (speed * delay); // 10m/s in milis
	double stepy = (endy - starty) / (speed * delay);

	// The per-loop step, corresponding to the velocity of the platform.
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
	spline.setLineColor(255, 128, 255);

	DrawConfig knots;
	knots.setType(DrawType::Cross);
	knots.setLineColor(255, 0, 255);

	ProfileDialog* pd = ProfileDialog::instance();
	pd->setBounds(0, tree.minz(), (end - start).norm(), tree.maxz());
	pd->addDrawConfig(&allPts);
	pd->addDrawConfig(&surf);
	pd->addDrawConfig(&spline);
	pd->addDrawConfig(&knots);
	pd->addDrawConfig(&alt);
	pd->addDrawConfig(&uav);
	pd->addDrawConfig(&knots);

	uav.data.emplace_back(0, altitude + offset);

	while(true) {

		plane = Eigen::Hyperplane<double, 3>(planeNorm, orig);
		line = Eigen::ParametrizedLine<double, 3>(orig, laserDir);

		// To clip off the points in the past.
		double dy = (orig - start).norm();
		gpf.setMinY(dy - 50.0);
		//std::cout << dy << "\n";

		ppf.setPlane(&plane);
		ppf.setLine(&line);

		tp.compute();

		std::list<Pt> salt;
		tp.splineAltitude(salt, 1);

		uav.data[0].first = dy;
		uav.data[0].second = altitude + offset;
		alt.data.emplace_back(dy, altitude + offset);

		spline.data.clear();
		for(const Pt& pt : salt)
			spline.data.emplace_back(pt.y(), pt.z());
		allPts.data.clear();
		for(const Pt& pt : tp.allPoints())
			allPts.data.emplace_back(pt.y(), pt.z());
		surf.data.clear();
		for(const Pt& pt : tp.surface())
			surf.data.emplace_back(pt.y(), pt.z());
		knots.data.clear();
		for(const Pt& pt : tp.knots())
			knots.data.emplace_back(pt.y(), pt.z());

		std::cout << tp.lastY() - dy << "\n";

		if(!tp.getTrajectoryAltitude(dy, altitude)) {
			//std::cerr << "Couldn't get new altitude.";
		} else {
			//std::cout << "Altitude: " << altitude << "\n";


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
