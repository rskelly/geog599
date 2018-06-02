

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

#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"
#include "sim/terrain.hpp"
#include "viewer/sim1viewer.hpp"

#include "util.hpp"
#include "sim/simulator.hpp"

using namespace uav::util;
using namespace uav::sim;

double time() {
	timeval time;
	gettimeofday(&time, NULL);
	return (double) time.tv_sec + ((double) time.tv_usec / 1000000);
}



void doRun(Simulator* sim) {
	sim->run();
}

Simulator::Simulator() : m_running(false) {}

void Simulator::start() {
	m_running = true;
	m_thread.reset(new std::thread(doRun, this));
}

void Simulator::stop() {
	m_running = false;
	m_thread->join();
}

void Simulator::setTerrainFile(const std::string& file) {
	RangeBridge::setTerrainFile(file);
}

void Simulator::run() {
	std::cerr << std::setprecision(12);
	double start = time();
	while(m_running) {
		double current = time() - start;
		m_platform.update(current);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
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
	sim.setTerrainFile("/home/rob/Documents/git/msc/experiments/simulation/data/pad_clip.tif");
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
