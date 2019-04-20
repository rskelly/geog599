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

#include "geog599/ConcaveHull.hpp"
#include "geog599/TrajectoryPlanner.hpp"
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
using namespace Eigen;

/// Convert degrees to radians.
double _rad(double deg) {
	return deg * M_PI / 180.0;
}

/// Create a rotation matrix using the Euler angles.
Matrix3d rotationMatrix(double rotX, double rotY, double rotZ) {
	double sx = std::sin(rotX), cx = std::cos(rotX);
	double sy = std::sin(rotY), cy = std::cos(rotY);
	double sz = std::sin(rotZ), cz = std::cos(rotZ);
	Matrix3d rotx; rotx << 1, 0, 0, 0, cx, -sx, 0, sx, cx;
	Matrix3d roty; roty << cy, 0, sy, 0, 1, 0, -sy, 0, cy;
	Matrix3d rotz; rotz << cz, -sz, 0, sz, cz, 0, 0, 0, 1;
	return rotz * roty * rotx;
}

/// Map of pre-prepared configurations.
std::unordered_map<std::string, PipelineConfig> configs;

// Prepared configurations.
void initConfigs() {
	configs.emplace("nrcan_1", PipelineConfig("data/nrcan_4_2m.txt", 305, 10, 0.5, 1, 2, _rad(5.7)));
	configs.emplace("swan_1", PipelineConfig("data/swan_lk_1_2m.txt", 25, 10, 0.02, 1, 10, _rad(5.7)));
	configs.emplace("swan_2", PipelineConfig("data/swan_lk_2_2m.txt", 40, 10, 0.002315, 0, 10, _rad(5.7)));
	configs.emplace("mt_doug_1", PipelineConfig("data/mt_doug_1_2m.txt", 90, 10, 0.002315, 0, 10, _rad(5.7)));
	configs.emplace("mt_doug_2", PipelineConfig("data/mt_doug_2_2m.txt", 100, 10, 20, 1, 20, _rad(5.7)));
	configs.emplace("bart_1", PipelineConfig("data/VITI_D168_BART_sess12_v1_1_2m.txt", 304, 10, 0.002610, 0, 5, _rad(5.7)));
}

/// Run the application.
void run(const std::string* configName, ProfileDialog* dlg) {


	//
	// Retrieve the config if extant.
	//

	if(configs.find(*configName) == configs.end())
		throw std::runtime_error("No configuration for '" + *configName + "'.");

	const PipelineConfig& config = configs.at(*configName);

	//
	// Load points and retrieve the Octree.
	//

	std::cout << "Loading points from " << config.filename << "\n";
	std::string ptsFile = config.filename;
	ProfilePointSource<Pt> tps(ptsFile);
	const Octree<Pt>& tree = tps.octree();

	//
	// Configure the coordinate bounds and backstep distances based on the octree.
	//

	double backStepX = 0, backStepY = 0;
	double viewx0 = 0, viewx1 = 0;
	double startx, endx, starty, endy;
	if(tree.width() > tree.length()) {
		startx = tree.minx();
		endx = tree.maxx();
		starty = tree.midy();
		endy = tree.midy();
		backStepX = 100;
		viewx1 = endx;
		viewx0 = startx;
	} else {
		startx = tree.midx();
		endx = tree.midx();
		starty = tree.miny();
		endy = tree.maxy();
		backStepY = 100;
		viewx1 = endy;
		viewx0 = starty;
	}

	//
	// Configure the matrices and vectors for navigation, point selectin
	// (via the hyper plane), laser rotation, etc.
	//

	std::cout << "Configuring matrices\n";

	// Offset from trajectory, and vehicle altitude (start altitude + offset)
	double altitude = config.startAltitude + config.altitude;

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
		planeNorm = planeRot * laserDir;
		planeNorm.normalize();
	}

	//
	// Configure the various processing objects.
	//

	tps.setNormal(planeNorm);
	tps.setMaxDist(MAX_DIST);

	Pt startPt(startx, starty, altitude);
	TrajectoryPlanner<Pt> tp(5);
	tp.setSmooth(config.smooth);
	tp.setWeight(config.weight);
	tp.setStartPoint(startPt);
	tp.setAlpha(config.alpha);
	tp.setPointSource(&tps);

	//
	// Configure the drawing objects and dialog window.
	//

	std::cout << "Drawing setup\n";

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
	pd->setBounds(viewx0, tree.minz(), viewx1, tree.maxz());
	pd->addDrawConfig(&allPts);
	pd->addDrawConfig(&surf);
	pd->addDrawConfig(&spline);
	pd->addDrawConfig(&knots);
	pd->addDrawConfig(&alt);
	pd->addDrawConfig(&knots);
	pd->addDrawConfig(&uav);

	//
	// Prepare some variables for controlling forward motion, etc.
	//

	std::cout << "Starting\n";

	double distance = (end - start).norm();
	double speed = 5.0; // m/s
	int delay = 500;	// 5 ms

	double stepx = (endx - startx) / distance * (speed / delay); // m/s in milis
	double stepy = (endy - starty) / distance * (speed / delay);

	Vector3d step(stepx, stepy, 0);
	Vector3d backstep(-backStepX, -backStepY, 0);
	orig += backstep;
	start += backstep;

	// Set the initial altitude on the UAV's position info.
	uav.data().emplace_back(0, altitude);

	// Main loop.
	while(!dlg->done) {

		// Advance the UAV position by the step amount.
		orig += step;

		// Points prior to this are finalized.
		double dy = (orig - start).norm() - backStepY;

		// Set the origin of the hyperplane.
		tps.setOrigin(orig);

		// Compute the current point cloud and trajectory.
		// If this fails, rest and skip.
		if(!tp.compute(Pt(0, dy + backStepY, 0))) {
			usleep(delay);
			continue;
		}

		// For tracking the UAV's past altitude.
		std::list<Pt> salt;
		tp.splineAltitude(salt, .5);

		// Get the spline's altitude at the current point.
		double dz;
		if(!tp.splineAltitude(dy, dz) || std::isnan(dz)) {
			dz = config.startAltitude + config.altitude;
		}

		// Set the current altitude on the UAV.
		{
			std::lock_guard<std::mutex> lk(uav.mtx);
			uav.data()[0].first = dy;
			uav.data()[0].second = dz + config.altitude;
		}

		// Set the current altitude on the UAV's elevation history (for drawing).
		{
			std::lock_guard<std::mutex> lk(alt.mtx);
			alt.data().emplace_back(dy, dz + config.altitude);
		}

		// Set the altitude of the drawable spline.
		if(true) {
			std::lock_guard<std::mutex> lk(spline.mtx);
			spline.data().clear();
			knots.data().clear();
			for(const Pt& pt : salt) {
				spline.data().emplace_back(pt.y(), pt.z());
				knots.data().emplace_back(pt.y(), pt.z());
			}
		}

		// Get the locations of all current and past input points.
		{
			std::lock_guard<std::mutex> lk(allPts.mtx);
			allPts.data().clear();
			for(const Pt& pt : tp.allPoints())
				allPts.data().emplace_back(pt.y(), pt.z());
		}

		// Prepare the concave hull surface.
		if(true){
			std::lock_guard<std::mutex> lk(surf.mtx);
			surf.data().clear();
			for(const Pt& pt : tp.surface())
				surf.data().emplace_back(pt.y(), pt.z());
		}

		double newalt;
		if(!tp.getTrajectoryAltitude(dy, newalt) || std::isnan(newalt)) {
			//std::cerr << "Couldn't get new altitude.";
		} else {
			//std::cout << "Pos: " << dy << ", " << altitude + config.altitude << "\n";

			/*
			altitude = newalt;
			altitude += config.altitude;
			orig[2] = altitude;
			start[2] = altitude;
			end[2] = altitude;
			*/
		}

		pd->draw();

		usleep(delay);

	}

	std::cerr << "Done\n";
}

int runGui(int argc, char** argv) {

	initConfigs();

	if(argc < 2) {
		std::cerr << "Usage: pipeline <config name>\n";
		std::cerr << "Available configurations:\n";
		for(const auto& it : configs)
			std::cerr << " - " << it.first << "\n";
		return 1;
	}

	std::string configName = argv[1];

	QApplication app(argc, argv);
	QDialog w;
	ProfileDialog p;
	p.setupUi(&w);
	w.show();
	std::thread t(&run, &configName, &p);
	app.exec();
	if(t.joinable())
		t.join();

	return 0;
}

int main(int argc, char** argv) {

	return runGui(argc, argv);

}
