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
#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"

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
	gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);
}

void RenderWidget::paintGL() {
	if(!m_initialized || !m_terrain || !m_platform) return;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	renderTerrain();
	renderPlatform();
	renderLaser();
}
void RenderWidget::initializeGL() {
	initializeOpenGLFunctions();
	glClearColor(0,0,0,1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	m_initialized = true;
}

void RenderWidget::renderLaser() {
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

	double size = 0.35 / trans[1];

	glBegin(GL_TRIANGLES);
	glColor3f(0.0, 0.0, 1.0);
	glVertex3f(x - size, y - size, z);
	glVertex3f(x + size, y, z);
	glVertex3f(x - size, y + size, z);
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

	for(size_t i = 0; i < vertices.size(); i += 3) {
		double x = std::abs(vertices[i + 0] - trans[0]) / width - 0.5;
		double y = std::abs(vertices[i + 1] - trans[3]) / width - 0.5;
		double z = (vertices[i + 2] - minz) / (maxz - minz) / 10.0; // Reduce vertical exaggeration; negate (up is negative).
		glColor3f(0.0, z * 10, 0.0);
		glVertex3f(x, y, z);
	}

	glEnd();
}

void RenderWidget::setTerrain(uav::sim::Terrain* terrain) {
	m_terrain = terrain;
}

void RenderWidget::setPlatform(uav::sim::Platform* platform) {
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
	std::cerr << "simupdate\n";
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

	glPanel->setTerrain(&(m_sim->terrain()));
	glPanel->setPlatform(&(m_sim->platform()));

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
	// TODO: Remember last dir.
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

