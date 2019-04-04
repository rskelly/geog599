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



#include <QtWidgets/QApplication>

#include "geog599/TrajectoryPlanner.hpp"
#include "geog599/HullPointFilter.hpp"
#include "geog599/GeomPointFilter.hpp"
#include "geog599/PlaneFilter.hpp"
#include "geog599/PointSorter.hpp"
#include "geog599/ProfilePointSource.hpp"

#include "ui/profile.hpp"
#include "ui/drawconfig.hpp"

#define MAX_RANGE 100 // The maximum laser range
#define MAX_DIST .1 // The maximum distance between a point and the search plane.
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
	double altitude;
	double smooth;
	double weight;
	double alpha;
	double laserAngle;

	PipelineConfig() :
		PipelineConfig("", 0, 0, 0, 0, 0, 0) {}

	PipelineConfig(const std::string& filename, double startAltitude, double altitude,
			double smooth, double weight, double alpha, double laserAngle) :
		filename(filename),
		startAltitude(startAltitude), altitude(altitude),
		smooth(smooth), weight(weight), alpha(alpha),
		laserAngle(laserAngle) {}

};

void run(ProfileDialog* dlg) {

	// Some pre-prepared configurations.
	std::unordered_map<std::string, PipelineConfig> configs;
	configs.emplace("nrcan_1", PipelineConfig("/home/rob/Documents/msc/data/lidar/nrcan_4.las", 305, 10, 15, 1, 15, _rad(5.7)));
	configs.emplace("mt_doug_1", PipelineConfig("/home/rob/Documents/msc/data/lidar/mt_doug_1.las", 80, 10, 5, 1, 15, _rad(5.7)));
	configs.emplace("mt_doug_2", PipelineConfig("/home/rob/Documents/msc/data/lidar/source/swan_lk.las", 80, 10, 5, 1, 15, _rad(5.7)));
	configs.emplace("bart_1", PipelineConfig("/home/rob/Documents/msc/data/lidar/2m_swath/VITI_D168_BART_sess12_v1_2_2m.txt", 316, 10, 5, 1, 2, _rad(5.7)));

	const PipelineConfig& config = configs["bart_1"];

	std::string ptsFile = config.filename;

	ProfilePointSource<Pt> tps(ptsFile);
	const Octree<Pt>& tree = tps.octree();

	double startx, endx, starty, endy;
	if(tree.width() > tree.length()) {
		startx = tree.minx();
		endx = tree.maxx();
		starty = tree.midy();
		endy = tree.midy();
	} else {
		startx = tree.midx();
		endx = tree.midx();
		starty = tree.miny();
		endy = tree.maxy();
	}

	// Offset from trajectory, and vehicle altitude (start altitude + offset)
	double altitude = config.startAltitude;


	// The start/origin and end points. These are determined by the point cloud.
	Vector3d orig(startx, starty, altitude);
	Vector3d start(startx, starty, altitude);
	Vector3d end(endx, endy, altitude);
	Vector3d planeNorm;

	{
		// The direction of the vehicle's travel and the direction of the x-axis (orthogonal).
		Vector3d direction = (end - start).normalized();
		Vector3d xaxis(direction[1], -direction[0], direction[2]);
		//std::cout << direction << ", " << xaxis << "\n";

		// The laser rotation matrix and the plane rotation matrix (orthogonal).
		Matrix3d laserRot = AngleAxisd(-config.laserAngle, xaxis).matrix();
		Matrix3d planeRot = AngleAxisd(-config.laserAngle + M_PI / 2.0, xaxis).matrix();
		//std::cout << laserRot << ", " << planeRot << "\n";

		// The laser direction and the plane normal.
		Vector3d laserDir = laserRot * direction;
		planeNorm = Vector3d(0, 1, 0);//planeRot * laserDir;
		planeNorm.normalize();
	}

	tps.setNormal(planeNorm);
	tps.setMaxDist(MAX_DIST);

	// The concave hull point filter.
	HullPointFilter<Pt> hpf(config.alpha);
	//GeomPointFilter<Pt> gpf;
	//gpf.setNextFilter(&hpf);

	Pt startPt(startx, starty, altitude);
	TrajectoryPlanner<Pt> tp;
	tp.setSmooth(config.smooth);
	tp.setWeight(config.weight);
	tp.setPointFilter(&hpf);
	tp.setStartPoint(startPt);
	tp.setPointSource(&tps);
	tp.spline().setXIndex(1);	// Set the indices on on the point object for y/z
	tp.spline().setYIndex(2);

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

	double speed = 10.0; // m/s
	int delay = 100;	// 1 ms

	double stepx = (endx - startx) / (speed * delay); // 10m/s in milis
	double stepy = (endy - starty) / (speed * delay);

	Vector3d step(stepx, stepy, 0);

	uav.data.emplace_back(0, altitude);

	while(!dlg->done) {

		orig += step;

		// To clip off the points in the past.
		double dy = (orig - start).norm();
		//gpf.setMinY(dy - 100.0);
		//std::cout << "d: " << dy << "\n";

		tps.setOrigin(orig);

		if(!tp.compute()) {
			std::this_thread::yield();
		//	continue;
		}

		std::list<Pt> salt;
		tp.splineAltitude(salt, 200);

		uav.data[0].first = 0;
		uav.data[0].second = altitude;
		alt.data.emplace_back(dy, altitude);

		spline.data.clear();
		for(const Pt& pt : salt)
			spline.data.emplace_back(pt.y(), pt.z());
		allPts.data.clear();
		for(const Pt& pt : tp.points())
			allPts.data.emplace_back(pt.y(), pt.z());
		surf.data.clear();
		for(const Pt& pt : tp.surface())
			surf.data.emplace_back(pt.y(), pt.z());
		knots.data.clear();
		for(const Pt& pt : tp.knots())
			knots.data.emplace_back(pt.y(), pt.z());

		std::cout << tp.lastY() - dy << "\n";

		if(!tp.getTrajectoryAltitude(dy, altitude)) {
			std::cerr << "Couldn't get new altitude.";
		} else {
			std::cout << "Altitude: " << altitude << "\n";

			altitude += config.altitude;
			orig[2] = altitude;
			start[2] = altitude;
			end[2] = altitude;
		}

		pd->draw();

	}

	std::cerr << "Done\n";
}

void runGui(int argc, char** argv) {
	QApplication app(argc, argv);
	QDialog w;
	ProfileDialog p;
	p.setupUi(&w);
	w.show();
	std::thread t(&run, &p);
	app.exec();
	if(t.joinable())
		t.join();
}

int main(int argc, char** argv) {

	runGui(argc, argv);

}
