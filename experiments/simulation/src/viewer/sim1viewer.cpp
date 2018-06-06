/*
 * sim1viewer.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

#include <iostream>
#include <vector>

#include <QtWidgets/QFileDialog>
#include <QtCore/QString>
#include <QtCore/QSettings>

#include <GL/gl.h>
#include <GL/glu.h>

#include "viewer/sim1viewer.hpp"
#include "viewer/renderwidget.hpp"
#include "sim/geometry.hpp"
#include "rangefinder.hpp"
#include "platform.hpp"

#define K_TERRAIN_FILE "terrainFile"

using namespace uav::viewer;
using namespace uav::sim;

RenderWidget::RenderWidget(QWidget* parent) :
	QOpenGLWidget(parent),
	m_initialized(false),
	m_terrain(nullptr),
	m_platform(nullptr) {
}

void RenderWidget::resizeGL(int w, int h) {
	//QOpenGLWidget::resizeGL(w, h);
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(5, (float) w / h, 0.01, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, -6, 2, 0, 0, 0, 0, 1, 0);
}

const GLfloat ambient[] = { 0.2, 0.2, 0.2, 1.0 };
const GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
const GLfloat diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
const GLfloat position[] = { 0.0, 0.0, 1.0, 1.0};

const GLfloat terrain_shininess[1] = {60.0f};
const GLfloat terrain_color[4] = {0.1f, 1.0f, 0.3f, 1.0f};
const GLfloat terrain_specular[] = { 1.0, 1.0, 1.0, 1.0 };

void RenderWidget::paintGL() {
	if(!m_initialized || !m_terrain || !m_platform) return;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_COLOR_MATERIAL);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, terrain_color);
	glMaterialfv(GL_FRONT, GL_SPECULAR, terrain_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, 50); //The shininess parameter

	renderTerrain();

	glEnable(GL_COLOR_MATERIAL);

	renderPlatform();

	renderLaser();
}


void RenderWidget::initializeGL() {
	initializeOpenGLFunctions();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
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

	double minz = m_terrain->minz();
	double maxz = m_terrain->maxz();
	double width = m_terrain->width();
	const double* trans = m_terrain->transform();
	std::vector<double> vertices;
	m_terrain->getVertices(vertices);

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
}

void RenderWidget::setTerrain(uav::sim::Terrain* terrain) {
	m_terrain = terrain;
}

void RenderWidget::setPlatform(uav::Platform* platform) {
	m_platform = platform;
}


Sim1Viewer::Sim1Viewer() :
		m_sim(nullptr),
		m_form(nullptr) {
	QSettings settings("sim1", "dijital.ca");
	m_settings[K_TERRAIN_FILE] = settings.value(K_TERRAIN_FILE, "").toString().toStdString();
}

void Sim1Viewer::setupUi(QDialog *Sim1Viewer) {
	Ui::Sim1Viewer::setupUi(Sim1Viewer);

	txtTerrainFile->setText(QString::fromStdString(m_settings[K_TERRAIN_FILE]));

	connect(txtTerrainFile, SIGNAL(textEdited(QString)), this, SLOT(terrainFileChanged(QString)));
    connect(btnTerrainFile, SIGNAL(clicked()), this, SLOT(btnTerrainFileClicked()));
    connect(btnClose, SIGNAL(clicked()), this, SLOT(btnCloseFormClicked()));
    connect(btnStart, SIGNAL(clicked()), this, SLOT(btnStartClicked()));
    connect(btnStop, SIGNAL(clicked()), this, SLOT(btnStopClicked()));
}

void Sim1Viewer::setSimulator(Simulator& sim) {
	m_sim = &sim;
	m_sim->addObserver(this);
}

void Sim1Viewer::simUpdate(Simulator& sim) {
	glPanel->update();
}

void Sim1Viewer::showForm() {
	if(!m_form)
		m_form = new QDialog();
	this->setupUi(m_form);
	m_form->show();
}

// slots

void Sim1Viewer::terrainFileChanged(QString file) {
	m_settings[K_TERRAIN_FILE] = file.toStdString();
}

void Sim1Viewer::btnStartClicked() {
	if(!m_sim)
		throw std::runtime_error("Simulator not set.");
	m_sim->setTerrainFile(m_settings[K_TERRAIN_FILE]);
	m_sim->addObserver(this);
	m_sim->start();

	glPanel->setTerrain(m_sim->terrain());
	glPanel->setPlatform(m_sim->platform());

	btnStart->setEnabled(false);
	btnStop->setEnabled(true);
}

void Sim1Viewer::btnStopClicked() {
	if(m_sim) {
		m_sim->stop();
		btnStart->setEnabled(true);
		btnStop->setEnabled(false);
	}
}

void Sim1Viewer::btnTerrainFileClicked() {
	QString file = QFileDialog::getOpenFileName(m_form, "Choose a Terrain File", "", "*.tif");
	terrainFileChanged(file);

}

void Sim1Viewer::btnCloseFormClicked() {
	if(m_form) {
		m_form->close();
		delete m_form;
	}
}


Sim1Viewer::~Sim1Viewer() {
	QSettings settings("sim1", "dijital.ca");
	for(const auto& item : m_settings)
		settings.setValue(QString::fromStdString(item.first), QString::fromStdString(item.second));
	if(m_form)
		delete m_form;
}

