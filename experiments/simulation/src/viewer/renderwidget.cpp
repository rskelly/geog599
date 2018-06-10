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

const GLfloat ambient[] = { 0.5, 0.5, 0.5, 1.0 };
const GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
const GLfloat diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
const GLfloat position[] = { -1.0, 0.0, 1.0, 1.0};

const GLfloat terrain_shininess[1] = {10.0f};
const GLfloat terrain_diffuse[4] = {0.2f, 1.0f, 0.2f, 1.0f};
const GLfloat terrain_ambient[4] = {0.2f, 1.0f, 0.2f, 1.0f};
const GLfloat terrain_specular[] = { 0.0, 0.0, 0.0, 1.0 };


RenderWidget::RenderWidget(QWidget* parent) :
	QOpenGLWidget(parent),
	m_initialized(false),
	m_fov(8),
	m_width(0), m_height(0),
	m_mouseX(0), m_mouseY(0),
	m_rotX(0), m_rotY(0),
	m_eyeDist(5),
	m_terrain(nullptr),
	m_platform(nullptr),
	m_surface(nullptr) {

	m_eyePos << m_eyeDist, 0, 0;
}

void RenderWidget::resizeGL(int w, int h) {
	//QOpenGLWidget::resizeGL(w, h);
	m_width = w;
	m_height = h;
	glViewport(0, 0, m_width, m_height);
}

void RenderWidget::paintGL() {
	if(!m_initialized || !m_terrain || !m_platform) return;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	Eigen::Matrix3d rotz = uav::util::rotateZ(m_rotX * PI * 2 - PI);
	Eigen::Matrix3d roty = uav::util::rotateY(m_rotY * PI / 3);
	m_eyePos = (rotz * roty) * Eigen::Vector3d(m_eyeDist, 0, 0);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(m_fov, (float) m_width / m_height, 0.01, 100.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// The eye position needs to be reversed from looking out from origin to looking back to origin.
	Eigen::Vector3d eyePos = m_eyePos * -1;
	gluLookAt(eyePos[0], eyePos[1], eyePos[2], 0, -0.25, 0, 0, 0, 1);
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
	const Eigen::Vector3d& dir = m_platform->rangefinderState().laserDirection();
	const Eigen::Vector3d& pos = m_platform->rangefinderState().laserPosition();

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
	const Eigen::Vector3d& pos = m_platform->platformState().position();

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

}

void RenderWidget::rotate(double x, double y) {
	m_rotX = std::max(-1.0, std::min(1.0, m_rotX + x));
	m_rotY = std::max(-1.0, std::min(1.0, m_rotY + y));
}

void RenderWidget::zoom(double delta) {
	if(delta > 0) {
		m_eyeDist += 0.1;
	} else if(delta < 0) {
		m_eyeDist -= 0.1;
	}
}

bool RenderWidget::event(QEvent* evt) {
	if(evt->type() == QEvent::MouseButtonPress) {
		QMouseEvent* mevt = dynamic_cast<QMouseEvent*>(evt);
		m_mouseX = mevt->globalX();
		m_mouseY = mevt->globalY();
	} else	if(evt->type() == QEvent::MouseMove) {
		QMouseEvent* mevt = dynamic_cast<QMouseEvent*>(evt);
		int mx = mevt->globalX();
		int my = mevt->globalY();
		rotate((double) (mx - m_mouseX) / width(), (double) (my - m_mouseY) / height());
		m_mouseX = mx;
		m_mouseY = my;
	} else if(evt->type() == QEvent::Wheel) {
		QWheelEvent* wevt = dynamic_cast<QWheelEvent*>(evt);
		zoom((double) wevt->delta());
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
