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

#include "sim1viewer.hpp"
#include "renderwidget.hpp"
#include "sim/geometry.hpp"
#include "sim/rangefinder.hpp"

#define K_TERRAIN_FILE "terrainFile"

using namespace uav::viewer;

RenderWidget::RenderWidget(QWidget* parent) :
	QOpenGLWidget(parent),
	m_width(0), m_height(0),
	m_minz(0), m_maxz(0) {
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
	renderTriangulations();
}
void RenderWidget::initializeGL() {
	initializeOpenGLFunctions();
	glClearColor(0,0,0,1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
}

void RenderWidget::renderTriangulations() {
	for(const std::vector<double>& d : m_triangulations) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glBegin(GL_TRIANGLES);

		for(size_t i = 0; i < d.size(); i += 3) {
			double x = std::abs(d[i + 0] - m_transform[0]) / m_width - 0.5;
			double y = std::abs(d[i + 1] - m_transform[3]) / m_width - 0.5;
			double z = (d[i + 2] - m_minz) / (m_maxz - m_minz) / 10.0; // Reduce vertical exaggeration; negate (up is negative).
			glColor3f(0.0, z * 10, 0.0);
			glVertex3f(x, y, z);
		}
		glFlush();
		glEnd();
	}
}

void RenderWidget::setTransform(const double* trans) {
	for(int i = 0; i < 6; ++i)
		m_transform[i] = trans[i];
}

void RenderWidget::addTriangulation(const std::vector<double>& tri) {
	m_triangulations.push_back(tri);
}

void RenderWidget::setSize(double width, double height) {
	m_width = width;
	m_height = height;
}

void RenderWidget::setZRange(double minz, double maxz) {
	m_minz = minz;
	m_maxz = maxz;
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
	std::vector<double> vertices;
	RangeBridge::terrain()->getVertices(vertices);
	glPanel->addTriangulation(vertices);
	glPanel->setTransform(RangeBridge::terrain()->transform());
	glPanel->setSize(RangeBridge::terrain()->width(), RangeBridge::terrain()->height());
	glPanel->setZRange(RangeBridge::terrain()->minz(), RangeBridge::terrain()->maxz());
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

