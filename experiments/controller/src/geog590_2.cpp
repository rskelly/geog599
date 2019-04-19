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

#include "../include/geog599/ConcaveHull.hpp"
#include "geog599/TrajectoryPlanner.hpp"
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

void run(ProfileDialog* dlg) {

	// Some pre-prepared configurations.
	std::unordered_map<std::string, PipelineConfig> configs;
	configs.emplace("nrcan_1", PipelineConfig("/home/rob/Documents/msc/data/lidar/nrcan_4.las", 305, 10, 5, 1, 15, 5.7, 5));
	configs.emplace("mt_doug_1", PipelineConfig("/home/rob/Documents/msc/data/lidar/mt_doug_1.las", 80, 10, 5, 1, 15, 5.7, 5));

	const PipelineConfig& config = configs["nrcan_1"];

	std::string ptsFile = config.filename;

	LASPointSource<Pt> tps(ptsFile);
	const Octree<Pt>& tree = tps.octree();

	double startx, endx, starty, endy;
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

	// The start/origin and end points. These are determined by the point cloud.
	Vector3d orig(startx, starty, altitude);
	Vector3d start(startx, starty, altitude);
	Vector3d end(endx, endy, altitude);

	// The concave hull point filter.
	HullPointFilter<Pt> hpf(config.alpha);

	Pt startPt(startx, starty, altitude);
	TrajectoryPlanner<Pt> tp;
	tp.setSmooth(config.smooth);
	tp.setWeight(config.weight);
	tp.setPointFilter(&hpf);
	tp.setPointSource(&tps);
	tp.setStartPoint(startPt);


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

	double dy = (orig - start).norm();

	uav.data.emplace_back(0, altitude + offset);

	tp.compute();

	std::list<Pt> salt;
	tp.splineAltitude(salt, 1);

	uav.data[0].first = 0;
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

	while(!dlg->done)
		std::this_thread::yield();

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

class SplineProps {
public:
	double x, y, sigma, a, b, c, d;
	SplineProps(double x, double y, double sigma) :
		x(x), y(y), sigma(sigma), a(0), b(0), c(0), d(0) {}
};

using namespace Eigen;

/*
int _idx(int n, int i) {
	while(i < 0) i += n;
	return i % n;
}
*/

void quincunx(int n, double* u, double* v, double* w, double* q) {

	u[1] = 0;
	u[0] = 0;

	// Factorize
	for(int j = 2; j < n - 1; ++j) {
		u[j] = u[j] - u[j - 2] * std::sqrt(w[j - 2]) - u[j - 1] * std::sqrt(v[j - 1]);
		v[j] = (v[j] - u[j - 1] * v[j - 1] * w[j - 1]) / u[j];
		w[j] = w[j] / u[j];
	}

	// Forward substitution.
	for(int j = 2; j < n - 1; ++j)
		q[j] = q[j] - v[j - 1] * q[j - 1] - w[j - 2] * q[j - 2];

	for(int j = 1; j < n - 1; ++j)
		q[j] = q[j] / u[j];

	// Back substitution
	q[n + 1] = 0;
	q[n] = 0;
	for(int j = 1; j < n - 1; ++j)
		q[j] = q[j] - v[j] * q[j + 1] - w[j] * q[j + 2];

}

void test() {
// From "smoothing with cubic splines"

	SplineProps s[] = {SplineProps(0,1,2), SplineProps(1,1,1), SplineProps(2,6,1), SplineProps(3,5,1), SplineProps(4,4,1), SplineProps(5,9,1), SplineProps(6,8,1)};
	int n = 7;
	double lambda = 0.5;

	double h[n];
	double r[n];
	double f[n];
	double p[n];
	double q[n];
	double u[n];
	double v[n];
	double w[n];

	// Smoothing spline
	double mu = 2 * (1 - lambda) / (3 * lambda);

	h[0] = s[1].x - s[0].x;
	r[0] = 3 / h[0];

	for(int i = 1; i < n - 1; ++i) {
		h[i] = s[i + 1].x - s[i].x;
		r[i] = 3 / h[i];
		f[i] = -(r[i - 1] + r[i]);
		p[i] = 2 * (s[i + 1].x - s[i - 1].x);
		q[i] = 3 * (s[i + 1].y - s[i].y) / h[i] - 3 * (s[i].y - s[i - 1].y) / h[i - 1];
	}

	for(int i = 1; i < n - 1; ++i) {
		u[i] = std::sqrt(r[i - 1]) * s[i - 1].sigma + std::sqrt(f[i]) * s[i].sigma + std::sqrt(r[i]) * s[i + 1].sigma;
		u[i] = mu * u[i] + p[i];
		v[i] = f[i] * r[	i] * s[i].sigma + r[i] * f[i + 1] * s[i + 1].sigma;
		v[i] = mu * v[i] + h[i];
		w[i] = mu * r[i] * r[i + 1] * s[i + 1].sigma;
	}

	quincunx(n, u, v, w, q);

	// Spline parameters
	s[0].d = s[0].y - mu * r[0] * q[1] * s[0].sigma;
	s[1].d = s[1].y - mu * (f[1] * q[1] * r[1] * q[2]) * s[0].sigma;
	s[0].a = q[1] / (3 * h[0]);
	s[0].b = 0;
	s[0].c = (s[1].d - s[0].d) / h[0] - q[1] * h[0] / 3;
	r[0] = 0;

	for(int j = 1; j < n - 1; ++j) {
		s[j].a = (q[j + 1] - q[j])/ (3 * h[j]);
		s[j].b = q[j];
		s[j].c = (q[j] + q[j - 1]) * h[j - 1] + s[j - 1].c;
		s[j].d = r[j - 1] * q[j - 1] + f[j] * q[j] + r[j] * q[j + 1];
		s[j].d = s[j].y - mu * s[j].d * s[j].sigma;
	}
}

int main(int argc, char** argv) {

	runGui(argc, argv);
	//test();

}
