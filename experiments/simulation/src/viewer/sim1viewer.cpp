/*
 * sim1viewer.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

#include <iostream>

#include <QtWidgets/QFileDialog>
#include <QtCore/QString>
#include <QtCore/QSettings>

#include "sim1viewer.hpp"
#include "renderwidget.hpp"

#define K_TERRAIN_FILE "terrainFile"

using namespace uav::viewer;

RenderWidget::RenderWidget(QWidget* parent) :
	QOpenGLWidget(parent) {
}

void RenderWidget::resizeGL(int w, int h) {

}
void RenderWidget::paintGL() {

}
void RenderWidget::initializeGL() {

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

