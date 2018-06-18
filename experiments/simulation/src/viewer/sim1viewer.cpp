/*
 * sim1viewer.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

#include <iostream>
#include <vector>

#include <QtWidgets/QFileDialog>
#include <QtGui/QKeyEvent>
#include <QtCore/QString>

#include "uav.hpp"
#include "viewer/sim1viewer.hpp"
#include "viewer/renderwidget.hpp"
#include "geometry.hpp"
#include "rangefinder.hpp"
#include "platform.hpp"
#include "util.hpp"

#define K_TERRAIN_FILE "terrainFile"
#define K_SHOW_TERRAIN "showTerrain"
#define K_GIMBAL_ANGLE "gimbalAngle"
#define K_ORIGIN_X "originX"
#define K_ORIGIN_Y "originY"
#define K_EYEPOS_X "eyeRotX"
#define K_EYEPOS_Y "eyeRotY"
#define K_EYEPOS_Z "eyeRotZ"
#define K_EYEROT_X "eyeRotY"
#define K_EYEROT_Y "eyeRotZ"
#define K_EYEDIST "eyeDist"

using namespace uav::viewer;
using namespace uav::sim;
using namespace uav::util;


Sim1Viewer::Sim1Viewer() :
		m_running(false),
		m_lastUpdate(0),
		m_sim(nullptr),
		m_form(nullptr) {
}

void Sim1Viewer::setupUi(QDialog *Sim1Viewer) {
	Ui::Sim1Viewer::setupUi(Sim1Viewer);

	// Set the terrain file on the text box from the settings map.
	txtTerrainFile->setText(m_settings.value(K_TERRAIN_FILE, "").toString());
	// Show whether terrain will be rendered.
	chkShowTerrain->setChecked(m_settings.value(K_SHOW_TERRAIN, true).toBool());
	// Show the gimbal angle.
	spnGimbalAngle->setValue(m_settings.value(K_GIMBAL_ANGLE, QVariant(45.0)).toDouble());

	glPanel->showTerrain(m_settings.value(K_SHOW_TERRAIN, true).toBool());

	glPanel->setOrigin(Eigen::Vector3d(
			m_settings.value(K_ORIGIN_X, 0.0).toDouble(),
			m_settings.value(K_ORIGIN_Y, 0.0).toDouble(), 0));
	glPanel->setEyePosition(Eigen::Vector3d(
			m_settings.value(K_EYEPOS_X, 0.0).toDouble(),
			m_settings.value(K_EYEPOS_Y, 0.0).toDouble(),
			m_settings.value(K_EYEPOS_Z, 0.0).toDouble()));
	glPanel->setEyeRotation(Eigen::Vector3d(
			m_settings.value(K_EYEROT_X, -1.0).toDouble(),
			m_settings.value(K_EYEROT_Y, -1.0).toDouble(), 0));
	glPanel->setEyeDistance(m_settings.value(K_EYEDIST, 5).toDouble());

	// Connect events.
	connect(txtTerrainFile, SIGNAL(textEdited(QString)), this, SLOT(terrainFileChanged(QString)));
    connect(btnTerrainFile, SIGNAL(clicked()), this, SLOT(btnTerrainFileClicked()));
    connect(btnClose, SIGNAL(clicked()), this, SLOT(btnCloseFormClicked()));
    connect(btnStart, SIGNAL(clicked()), this, SLOT(btnStartClicked()));
    connect(btnStop, SIGNAL(clicked()), this, SLOT(btnStopClicked()));
    connect(chkShowInfo, SIGNAL(toggled(bool)), SLOT(chkShowInfoChanged(bool)));
    connect(chkShowSettings, SIGNAL(toggled(bool)), SLOT(chkShowSettingsChanged(bool)));
    connect(chkShowTerrain, SIGNAL(toggled(bool)), SLOT(chkShowTerrainChanged(bool)));
    connect(spnGimbalAngle, SIGNAL(valueChanged(double)), SLOT(spnGimbalAngleChanged(double)));
}

void Sim1Viewer::setSimulator(Simulator& sim) {
	m_sim = &sim;
}

void __update(Sim1Viewer* viewer, bool* running) {
	while(*running) {
		viewer->update();
		std::this_thread::sleep_for(std::chrono::milliseconds((int) (1.0 / 24.0 * 1000)));
	}
}

void Sim1Viewer::start() {
	if(!m_running) {
		if(!m_sim)
			throw std::runtime_error("Simulator not set.");
		m_sim->setTerrainFile(m_settings.value(K_TERRAIN_FILE, "").toString().toStdString());
		m_sim->setGimbalAngle(m_settings.value(K_GIMBAL_ANGLE, 0).toDouble() * PI / 180);

		glPanel->setTerrain(m_sim->terrain());
		glPanel->setPlatform(m_sim->platform());
		glPanel->setSurface(m_sim->platform()->surface());

		m_sim->start();

		m_running = true;
		m_thread = std::thread(__update, this, &m_running);

		btnStart->setEnabled(false);
		btnStop->setEnabled(true);
	}
}

void Sim1Viewer::stop() {
	if(m_running) {
		m_running = false;
		if(m_thread.joinable())
			m_thread.join();

		m_sim->stop();

		btnStart->setEnabled(true);
		btnStop->setEnabled(false);
	}
}

void Sim1Viewer::updateInfo() {
	double elev = m_sim->platform()->platformState().altitude();
	double prate = m_sim->platform()->platformState().pulseRate();
	txtElevation->setText(QString::number(elev, 'f', 2));
	txtPulseRate->setText(QString::number(prate, 'f', 2));
}

void Sim1Viewer::update() {
	glPanel->update();
	updateInfo();
}

void Sim1Viewer::showForm() {
	if(!m_form)
		m_form = new QDialog();
	this->setupUi(m_form);
	m_form->show();
}


// slots

void Sim1Viewer::terrainFileChanged(QString file) {
	m_settings.setValue(K_TERRAIN_FILE, file);
	txtTerrainFile->setText(file);
}

void Sim1Viewer::chkShowTerrainChanged(bool show) {
	glPanel->showTerrain(show);
	m_settings.setValue(K_SHOW_TERRAIN, show ? true : false);
}

void Sim1Viewer::chkShowInfoChanged(bool checked) {
	frmInfo->setVisible(checked);
}

void Sim1Viewer::chkShowSettingsChanged(bool checked) {
	frmSettings->setVisible(checked);
}

void Sim1Viewer::btnStartClicked() {
	btnStart->setEnabled(false);
	start();
}

void Sim1Viewer::btnStopClicked() {
	btnStop->setEnabled(false);
	stop();
}

void Sim1Viewer::btnTerrainFileClicked() {
	QString file = QFileDialog::getOpenFileName(m_form, "Choose a Terrain File", "", "*.tif");
	terrainFileChanged(file);
}

void Sim1Viewer::btnCloseFormClicked() {
	if(m_form) {
		m_form->close();
		delete m_form;
		m_form = nullptr;
	}
}

void Sim1Viewer::spnGimbalAngleChanged(double value) {
	m_settings.setValue(K_GIMBAL_ANGLE, QString::number(value));
}

Sim1Viewer::~Sim1Viewer() {
	const Eigen::Vector3d origin = glPanel->origin();
	m_settings.setValue(K_ORIGIN_X, origin[0]);
	m_settings.setValue(K_ORIGIN_Y, origin[1]);
	const Eigen::Vector3d eyePos = glPanel->eyePosition();
	m_settings.setValue(K_EYEPOS_X, eyePos[0]);
	m_settings.setValue(K_EYEPOS_Y, eyePos[1]);
	m_settings.setValue(K_EYEPOS_Z, eyePos[2]);
	const Eigen::Vector3d eyeRot = glPanel->eyeRotation();
	m_settings.setValue(K_EYEROT_X, eyeRot[0]);
	m_settings.setValue(K_EYEROT_Y, eyeRot[1]);
	m_settings.setValue(K_EYEDIST, glPanel->eyeDistance());
	if(m_form)
		delete m_form;
}

