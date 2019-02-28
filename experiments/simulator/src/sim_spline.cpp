

#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <vector>
#include <memory>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <QtCore/QObject>
#include <QtCore/QEvent>
#include <QtCore/QString>

#include <Eigen/Core>

#include "util.hpp"
#include "surface.hpp"
#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"
#include "sim/terrain.hpp"
#include "sim/simulator.hpp"
#include "sim/gimbal.hpp"
#include "viewer/sim1viewer.hpp"

#include "impl/rangefindersf30c.hpp"

using namespace uav::util;
using namespace uav::sim;
using namespace uav::surface;
using namespace uav::impl;

Simulator::Simulator() :
	m_running(false) {

	m_controller = new uav::sim::Controller();

	// Instantiate the terrain. Will load the DEM later.
	m_terrain = new Terrain();

	// These are parameters that organize the components of the laser system w/r/t the
	// UAV platform.
	Gimbal* gimbal = new SinGimbal();
	//NetGimbal* gimbal = new NetGimbal("10.0.0.16", 9000);

	//gimbal->setPosition(Eigen::Vector3d(0, 0, -0.02)); // The laser sits on a mount 2cm high, upside down.
	gimbal->setStaticPosition(Eigen::Vector3d(0.2, 0, -0.05)); // 20cm forward, 0cm to side, 5cm down
	gimbal->setStaticOrientation(Eigen::Vector3d(0, 0, 0)); // down (around the y axis.) TODO: This seems to be upside-down...

	// Set up the nadir rangefinder using a range bridge and the terrain.
	RangeBridge* rb1 = new RangeBridge();
	rb1->setTerrain(m_terrain);

	// Set up the forward rangefinder.
	Rangefinder* rangefinder = new Rangefinder();
	rangefinder->setRangeBridge(rb1);

	Rangefinder* nadirRangefinder = new Rangefinder();
	nadirRangefinder->setRangeBridge(rb1);
	nadirRangefinder->setPulseFrequency(10);

	// This is the surface reconstruction module.
	//DelaunaySurface* surface = new DelaunaySurface();
	AlphaSurface* surface = new AlphaSurface();
	surface->setAlpha(0);

	// Configure the platform.
	m_platform = new Platform();
	m_platform->setGimbal(gimbal);
	m_platform->setSurface(surface);
	m_platform->setRangefinder(rangefinder);
	m_platform->setNadirRangefinder(nadirRangefinder);

	double startx = 620154.92241;
	double starty = 5613102.36059;
	double endx = 620195.30108;
	double endy = 5613635.35912;
	double altitude = 310;

	uav::sim::PlatformState state(dynamic_cast<const uav::sim::PlatformState&>(m_platform->platformState()));
	state.setOrientation(Eigen::Vector3d(startx, starty, altitude));
	state.setLinearVelocity(Eigen::Vector3d(0, 4, 0));
	m_platform->setInitialPlatformState(state);

	m_controller->setPlatform(m_platform);

}

void Simulator::start() {
	if(!m_running) {
		m_controller->start();
		m_running = true;
	}
}

void Simulator::stop() {
	if(m_running) {
		m_running = false;
		m_controller->stop();
	}
}

void Simulator::setTerrainFile(const std::string& file, int band) {
	m_terrain->load(file, band);
	double x = m_terrain->minx() + 10;
	double y = m_terrain->miny() + m_terrain->height() / 2.0;

	// Start the vehicle off at 10cm above the terrain.
	uav::sim::PlatformState state(dynamic_cast<const uav::sim::PlatformState&>(m_platform->platformState()));
	state.setPosition(Eigen::Vector3d(x, y, m_terrain->sample(x, y) + 10));
	m_platform->setInitialPlatformState(state);
}

uav::sim::Terrain* Simulator::terrain() {
	return m_terrain;
}

uav::Platform* Simulator::platform() {
	return m_platform;
}

Simulator::~Simulator() {
	stop();
	delete m_terrain;
	delete m_platform;
}

int runWithGui(int argc, char **argv) {

	class Sim1Application : public QApplication {
	public:
		Sim1Application(int &argc, char **argv) : QApplication(argc, argv) {
			QCoreApplication::setOrganizationName("dijital.ca");
			QCoreApplication::setOrganizationDomain("dijital.ca");
			QCoreApplication::setApplicationName("uavsim1");
		}

		bool notify(QObject *receiver, QEvent *e) {
			try {
				return QApplication::notify(receiver, e);
			} catch(const std::exception &ex) {
				std::cerr << ex.what() << "\n";
				QMessageBox err;
				err.setText("Error");
				err.setInformativeText(QString(ex.what()));
				bool result = err.exec();
				return result;
			}
		}
	};

	Sim1Application q(argc, argv);
	uav::viewer::Sim1Viewer v;
	Simulator sim;
	v.setSimulator(sim);
	v.showForm();
	bool result = q.exec();
	return result;

}

int main(int argc, char** argv) {

	//testMath();
	//Terrain* t = loadTerrain();
	//initWindow();
	//renderTerrain(t);
	//runWindow();

	/*
	Platform p;

	std::cerr << std::setprecision(12);

	double start = time();

	while(true) {
		double current = time() - start;

		p.update(current);

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	*/

	runWithGui(argc, argv);
}
