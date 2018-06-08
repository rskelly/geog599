/*
 * renderwidget.cpp
 *
 *  Created on: Jun 7, 2018
 *      Author: rob
 */

#include <QtWidgets/QFileDialog>
#include <QtCore/QString>
#include <QtCore/QSettings>
#include <QtCore/QEvent>
#include <QtGui/QMouseEvent>

#include <Eigen/Core>

#include <GL/gl.h>
#include <GL/glu.h>

#include "uav.hpp"
#include "renderwidget.hpp"
#include "util.hpp"

using namespace uav::viewer;

RenderWidget::RenderWidget(QWidget* parent) :
	QOpenGLWidget(parent),
	m_initialized(false),
	m_terrain(nullptr),
	m_platform(nullptr),
	m_surface(nullptr) {
}

Eigen::Vector3d __look(-6, -6, 2);

void RenderWidget::resizeGL(int w, int h) {
	//QOpenGLWidget::resizeGL(w, h);
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(8, (float) w / h, 0.01, 100.0);
}

const GLfloat ambient[] = { 0.5, 0.5, 0.5, 1.0 };
const GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
const GLfloat diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
const GLfloat position[] = { -1.0, 0.0, 1.0, 1.0};

const GLfloat terrain_shininess[1] = {10.0f};
const GLfloat terrain_diffuse[4] = {0.2f, 1.0f, 0.2f, 1.0f};
const GLfloat terrain_ambient[4] = {0.2f, 1.0f, 0.2f, 1.0f};
const GLfloat terrain_specular[] = { 0.0, 0.0, 0.0, 1.0 };

void RenderWidget::paintGL() {
	if(!m_initialized || !m_terrain || !m_platform) return;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	std::cerr << "look 1: " << __look << "\n";
	//Eigen::Vector3d look = uav::util::eulerToVector(__look);
	//std::cerr << "look 2: " << look << "\n";
	//gluLookAt(-6, -6, 1, 0, 0, 0, 0, 0, 1);
	gluLookAt(__look[0], __look[1], __look[2], 0, 0, 0, 0, 1, 1);
	//glDisable(GL_COLOR_MATERIAL);
	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	//glLightfv(GL_LIGHT0, GL_POSITION, position);
	glEnable(GL_COLOR_MATERIAL);

	renderTerrain();


	renderPlatform();

	renderLaser();

	renderSurface();
}

void RenderWidget::initializeGL() {
	initializeOpenGLFunctions();
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);
	//glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);

	m_initialized = true;
}

void RenderWidget::renderLaser() {
	const Eigen::Vector3d& dir = m_platform->laserDirection();
	const Eigen::Vector3d& pos = m_platform->laserPosition();

	double minz = m_terrain->minz();
	double maxz = m_terrain->maxz();
	double width = m_terrain->width();
	const double* trans = m_terrain->transform();

	double x0 = std::abs(pos[0] - trans[0]) / width - 0.5;
	double y0 = std::abs(pos[1] - trans[3]) / width - 0.5;
	double z0 = (pos[2] - minz) / (maxz - minz) / 10.0; // Reduce vertical exaggeration; negate (up is negative).

	Eigen::Vector3d pos1 = pos + dir * pos[2];

	double x1 = std::abs(pos1[0] - trans[0]) / width - 0.5;
	double y1 = std::abs(pos1[1] - trans[3]) / width - 0.5;
	double z1 = (pos1[2] - minz) / (maxz - minz) / 10.0; // Reduce vertical exaggeration; negate (up is negative).

	glBegin(GL_LINES);
	glColor3f(1.0, 0.0, 0.0);
	glVertex3f(x0, y0, z0);
	glVertex3f(x1, y1, z1);
	glEnd();
}

void RenderWidget::renderPlatform() {
	const Eigen::Vector3d& pos = m_platform->position();

	double minz = m_terrain->minz();
	double maxz = m_terrain->maxz();
	double width = m_terrain->width();
	const double* trans = m_terrain->transform();

	double x = std::abs(pos[0] - trans[0]) / width - 0.5;
	double y = std::abs(pos[1] - trans[3]) / width - 0.5;
	double z = (pos[2] - minz) / (maxz - minz) / 10.0; // Reduce vertical exaggeration; negate (up is negative).

	double size = 0.2 / trans[1];

	glBegin(GL_QUADS);

	glColor3f(0.0, 0.0, 1.0);
	glVertex3f(x - size, y - size, z + size / 2);
	glVertex3f(x + size, y - size, z + size / 2);
	glVertex3f(x + size, y + size, z + size / 2);
	glVertex3f(x - size, y + size, z + size / 2);

	glVertex3f(x - size, y - size, z - size / 2);
	glVertex3f(x + size, y - size, z - size / 2);
	glVertex3f(x + size, y + size, z - size / 2);
	glVertex3f(x - size, y + size, z - size / 2);

	glColor3f(0.0, 1.0, 1.0);
	glVertex3f(x - size, y + size, z - size / 2);
	glVertex3f(x + size, y + size, z - size / 2);
	glVertex3f(x + size, y + size, z + size / 2);
	glVertex3f(x - size, y + size, z + size / 2);

	glVertex3f(x - size, y - size, z - size / 2);
	glVertex3f(x + size, y - size, z - size / 2);
	glVertex3f(x + size, y - size, z + size / 2);
	glVertex3f(x - size, y - size, z + size / 2);

	glColor3f(1.0, 1.0, 0.0);
	glVertex3f(x + size, y - size, z - size / 2);
	glVertex3f(x + size, y - size, z + size / 2);
	glVertex3f(x + size, y + size, z + size / 2);
	glVertex3f(x + size, y + size, z - size / 2);

	glVertex3f(x - size, y - size, z - size / 2);
	glVertex3f(x - size, y - size, z + size / 2);
	glVertex3f(x - size, y + size, z + size / 2);
	glVertex3f(x - size, y + size, z - size / 2);

	glEnd();
}

void RenderWidget::renderTerrain() {

	glBegin(GL_TRIANGLES);

	//glMaterialfv(GL_FRONT, GL_AMBIENT, terrain_ambient);
	//glMaterialfv(GL_FRONT, GL_DIFFUSE, terrain_diffuse);
	//glMaterialfv(GL_FRONT, GL_SPECULAR, terrain_specular);
	//glMaterialfv(GL_FRONT, GL_SHININESS, terrain_shininess); //The shininess parameter

	double minz = m_terrain->minz();
	double maxz = m_terrain->maxz();
	double width = m_terrain->width();
	const double* trans = m_terrain->transform();
	std::vector<double> vertices;
	m_terrain->getVertices(vertices);

	// TODO: Fix and use normals from terrain.

	double x[3];
	double y[3];
	double z[3];

	for(size_t i = 0, j = 0; i < vertices.size(); i += 3, ++j) {
		x[j % 3] = std::abs(vertices[i + 0] - trans[0]) / width - 0.5;
		y[j % 3] = std::abs(vertices[i + 1] - trans[3]) / width - 0.5;
		z[j % 3] = (vertices[i + 2] - minz) / (maxz - minz); // Reduce vertical exaggeration; negate (up is negative).
		if(j % 3 == 2) {
			//double nx = y[0] * z[1] - z[0] * y[1];
			//double ny = z[0] * x[1] - x[0] * z[1];
			//double nz = x[0] * y[1] - y[0] * x[1];
			//glNormal3f(nx, ny, nz);
			glColor3f(0.0f, 0.3 + z[0] * 0.5, 0.0f);
			glVertex3f(x[0], y[0], z[0] / 10 - 0.01); // Lower to make space for surface.
			glColor3f(0.0f, 0.3 + z[1] * 0.5, 0.0f);
			glVertex3f(x[1], y[1], z[1] / 10 - 0.01);
			glColor3f(0.0f, 0.3 + z[2] * 0.5, 0.0f);
			glVertex3f(x[2], y[2], z[2] / 10 - 0.01);
		}
	}

	glEnd();
}

void RenderWidget::renderSurface() {

	//glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_TRIANGLES);

	double minz = m_terrain->minz();
	double maxz = m_terrain->maxz();
	double width = m_terrain->width();
	const double* trans = m_terrain->transform();
	std::vector<double> vertices;
	dynamic_cast<uav::surface::DelaunaySurface*>(m_surface)->getVertices(vertices);

	double x[3];
	double y[3];
	double z[3];

	for(size_t i = 0, j = 0; i < vertices.size(); i += 3, ++j) {
		x[j % 3] = std::abs(vertices[i + 0] - trans[0]) / width - 0.5;
		y[j % 3] = std::abs(vertices[i + 1] - trans[3]) / width - 0.5;
		z[j % 3] = (vertices[i + 2] - minz) / (maxz - minz); // Reduce vertical exaggeration; negate (up is negative).
		if(j % 3 == 2) {
			double nx = y[0] * z[1] - z[0] * y[1];
			double ny = z[0] * x[1] - x[0] * z[1];
			double nz = x[0] * y[1] - y[0] * x[1];
			glNormal3f(nx, ny, nz);
			glVertex3f(x[0], y[0], z[0] / 10);
			glVertex3f(x[1], y[1], z[1] / 10);
			glVertex3f(x[2], y[2], z[2] / 10);
		}
	}

	glEnd();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//glEnable(GL_LIGHTING);
}

double __rotz = 0;

void RenderWidget::rotate(int x, int y) {
	__rotz += 0.1 * x;
	Eigen::Matrix3d rotz = uav::util::rotateZ(__rotz);
	__look = rotz * __look;
}

int __mx = -1;
int __my = -1;

bool RenderWidget::event(QEvent* evt) {
	if(evt->type() == QEvent::MouseMove) {
		QMouseEvent* mevt = dynamic_cast<QMouseEvent*>(evt);
		int mx = mevt->globalX();
		int my = mevt->globalY();
		if(__mx == -1) {
			__mx = mx;
			__my = my;
		} else {
			rotate(mx - __mx, my - __my);
			__mx = mx;
			__my = my;
		}
		std::cerr << "evt: mousemove\n";
	}
	return QWidget::event(evt);
}

void RenderWidget::setTerrain(uav::sim::Terrain* terrain) {
	m_terrain = terrain;
}

void RenderWidget::setPlatform(uav::Platform* platform) {
	m_platform = platform;
}

void RenderWidget::setSurface(uav::surface::Surface* surface) {
	m_surface = surface;
}
