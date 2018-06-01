

#include <iostream>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <iomanip>
#include <vector>

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

using namespace uav::util;
using namespace uav::sim;

double time() {
	timeval time;
	gettimeofday(&time, NULL);
	return (double) time.tv_sec + ((double) time.tv_usec / 1000000);
}


void testMath() {
	double a = toRad(45);
	Eigen::Vector3d vec(1, 0, 0);
	std::cerr << vec[0] << ", " << vec[1] << ", " << vec[2] << "\n";
	Eigen::Matrix3d mtx = rotFromAxisAngle(vec, a);
	std::cerr << mtx(0, 0) << ", " << mtx(0, 1) << ", " << mtx(0, 2) << "\n";
	std::cerr << mtx(1, 0) << ", " << mtx(1, 1) << ", " << mtx(1, 2) << "\n";
	std::cerr << mtx(2, 0) << ", " << mtx(2, 1) << ", " << mtx(2, 2) << "\n";
	Eigen::Vector3d input(2, 1, 3);
	std::cerr << input[0] << ", " << input[1] << ", " << input[2] << "\n";
	Eigen::Vector3d out = mtx * input;
	std::cerr << out[0] << ", " << out[1] << ", " << out[2] << "\n";
}

Terrain* loadTerrain() {
	return new Terrain("/home/rob/Documents/gis/geocat/srtm/n40_w082_1arc_v3_utm.tif");
}

/*
GLFWwindow* win;

bool initWindow() {

	if(!glfwInit())
		return false;

	 win = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
	 if (!win) {
		 glfwTerminate();
		 return false;
	 }

	glfwMakeContextCurrent(win);
	return true;
}

void runWindow() {

	while (!glfwWindowShouldClose(win)) {
		glClear(GL_COLOR_BUFFER_BIT);
		glfwSwapBuffers(win);
		glfwPollEvents();
	}

	glfwTerminate();

}

bool renderTerrain(Terrain* terrain) {

	std::vector<double> vertices;
	terrain->getVertices(vertices);

	unsigned int id;
	glGenBuffers(1, &id);
	return false;
}
*/

int runWithGui(int argc, char **argv) {
#ifdef WITH_GUI

	class Sim1Application : public QApplication {
	public:
		Sim1Application(int &argc, char **argv) : QApplication(argc, argv) {}
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
