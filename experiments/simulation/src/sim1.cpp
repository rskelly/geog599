

#include <iostream>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <iomanip>
#include <vector>
#include <memory>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <QtCore/QObject>
#include <QtCore/QEvent>
#include <QtCore/QString>

#include <Eigen/Core>

/*
#include <GL/glew.h>
#include <GL/glut.h>
#include <GLFW/glfw3.h>
*/

#include "util.hpp"
#include "surface.hpp"
#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"
#include "sim/terrain.hpp"
#include "sim/simulator.hpp"
#include "sim/gimbal.hpp"
#include "viewer/sim1viewer.hpp"

using namespace uav::util;
using namespace uav::sim;
using namespace uav::surface;

double time() {
	timeval time;
	gettimeofday(&time, NULL);
	return (double) time.tv_sec + ((double) time.tv_usec / 1000000);
}

void doRun(Simulator* sim) {
	sim->run();
}

Simulator::Simulator() :
	m_running(false) {

	m_gimbal = new SinGimbal(1, 1);
	m_gimbal->setPosition(Eigen::Vector3d(0, 0, -0.02)); // The laser sits on a mount 2cm high, upside down.
	m_gimbal->setStaticPosition(Eigen::Vector3d(0.2, 0, -0.05)); // 20cm forward, 0cm to side, 5cm down
	m_gimbal->setStaticOrientation(Eigen::Vector3d(0, PI / 4., 0)); // 45deg down (around the y axis.) TODO: This seems to be upside-down...

	m_terrain = new Terrain();

	m_platform = new Platform();
	m_platform->setGimbal(m_gimbal);
	m_platform->setRangefinder(new Rangefinder());

	m_surface = new DelaunaySurface();
}

void Simulator::start() {
	m_running = true;
	m_thread.reset(new std::thread(doRun, this));
}

void Simulator::stop() {
	m_running = false;
	m_thread->join();
}

void Simulator::setTerrainFile(const std::string& file) {
	m_terrain->load(file);
	RangeBridge::setTerrain(m_terrain);
}

uav::sim::Terrain* Simulator::terrain() {
	return m_terrain;
}

uav::Platform* Simulator::platform() {
	return m_platform;
}

uav::Gimbal* Simulator::gimbal() {
	return m_gimbal;
}

void Simulator::addObserver(SimulatorObserver* obs) {
	m_obs.push_back(obs);
}

void Simulator::run() {
	std::cerr << std::setprecision(12);
	double start = time();
	while(m_running) {
		double current = time() - start;
		m_platform->update(current);
		for(SimulatorObserver* obs : m_obs)
			obs->simUpdate(*this);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

Simulator::~Simulator() {
	delete m_terrain;
	delete m_platform;
	delete m_gimbal;
}

int runWithGui(int argc, char **argv) {
#ifdef WITH_GUI

	class Sim1Application : public QApplication {
	public:
		Sim1Application(int &argc, char **argv) : QApplication(argc, argv) {

		}

		bool notify(QObject *receiver, QEvent *e) {
			try {
				return QApplication::notify(receiver, e);
			} catch(const std::exception &ex) {
				QMessageBox err;
				err.setText("Error");
				err.setInformativeText(QString(ex.what()));
				err.exec();
				return false;
			}
		}
	};

	Sim1Application q(argc, argv);
	uav::viewer::Sim1Viewer v;
	Simulator sim;
	v.setSimulator(sim);
	v.showForm();
	return q.exec();
#else
	std::cerr << "GUI not enabled." << std::endl;
	return 1;
#endif
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
