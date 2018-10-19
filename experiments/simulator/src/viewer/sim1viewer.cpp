/*
 * sim1viewer.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

#include <iostream>
#include <vector>
#include <string>

#include <QtWidgets/QFileDialog>
#include <QtGui/QKeyEvent>
#include <QtCore/QString>

#include "uav.hpp"
#include "viewer/sim1viewer.hpp"
#include "viewer/renderwidget.hpp"
#include "sim/rangefinder.hpp"
#include "sim/gimbal.hpp"
#include "geometry.hpp"
#include "rangefinder.hpp"
#include "platform.hpp"
#include "util.hpp"

constexpr char K_TERRAIN_FILE[] = "terrainFile";
constexpr char K_TERRAIN_BAND[] = "terrainBand";
constexpr char K_SHOW_TERRAIN[] = "showTerrain";
constexpr char K_GIMBAL_ANGLE[] = "gimbalAngle";
constexpr char K_ORIGIN_X[] = "originX";
constexpr char K_ORIGIN_Y[] = "originY";
constexpr char K_EYEPOS_X[] = "eyePosX";
constexpr char K_EYEPOS_Y[] = "eyePosY";
constexpr char K_EYEPOS_Z[] = "eyePosZ";
constexpr char K_EYEROT_X[] = "eyeRotX";
constexpr char K_EYEROT_Y[] = "eyeRotY";
constexpr char K_EYEROT_Z[] = "eyeRotZ";
constexpr char K_EYEDIST[] = "eyeDist";
constexpr char K_SWEEP_ANGLE[] = "sweepAngle";
constexpr char K_SWEEP_FREQ[] = "sweepFreq";
constexpr char K_PULSE_FREQ[] = "pulseFreq";

constexpr double D_GIMBAL_ANGLE = 45;
constexpr int D_TERRAIN_BAND = 1;
constexpr double D_SWEEP_ANGLE = 45;
constexpr double D_SWEEP_FREQ = 2;
constexpr double D_PULSE_FREQ = 50;

using namespace uav::viewer;
using namespace uav::sim;
using namespace uav::util;


Sim1Viewer::Sim1Viewer() :
		m_running(false),
		m_lastUpdate(0),
		m_sim(nullptr),
		m_form(nullptr) {
}

void Sim1Viewer::applySettings() {
	// Set the terrain file on the text box from the settings map.
	txtTerrainFile->setText(m_settings.value(K_TERRAIN_FILE, "").toString());
	spnBand->setValue(m_settings.value(K_TERRAIN_BAND, D_TERRAIN_BAND).toInt());
	// Show whether terrain will be rendered.
	chkShowTerrain->setChecked(m_settings.value(K_SHOW_TERRAIN, true).toBool());
	// Show the gimbal angle.
	spnGimbalAngle->setValue(m_settings.value(K_GIMBAL_ANGLE, D_GIMBAL_ANGLE).toDouble());
	spnSweepFrequency->setValue(m_settings.value(K_SWEEP_FREQ, D_SWEEP_FREQ).toDouble());
	spnSweepAngle->setValue(m_settings.value(K_SWEEP_ANGLE, D_SWEEP_ANGLE).toDouble());
	spnPulseFrequency->setValue(m_settings.value(K_PULSE_FREQ, D_PULSE_FREQ).toDouble());

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
			m_settings.value(K_EYEROT_Y, -1.0).toDouble(),
			m_settings.value(K_EYEROT_Z, -1.0).toDouble()));
	glPanel->setEyeDistance(m_settings.value(K_EYEDIST, 5).toDouble());
}

void Sim1Viewer::setupUi(QDialog *Sim1Viewer) {
	Ui::Sim1Viewer::setupUi(Sim1Viewer);

	applySettings();

	// Connect events.
	connect(txtTerrainFile, SIGNAL(textEdited(QString)), this, SLOT(terrainFileChanged(QString)));
    connect(btnTerrainFile, SIGNAL(clicked()), this, SLOT(btnTerrainFileClicked()));
    connect(btnClose, SIGNAL(clicked()), this, SLOT(btnCloseFormClicked()));
    connect(btnStart, SIGNAL(clicked()), this, SLOT(btnStartClicked()));
    connect(btnStop, SIGNAL(clicked()), this, SLOT(btnStopClicked()));
    connect(btnReset, SIGNAL(clicked()), this, SLOT(btnResetClicked()));
    connect(chkShowInfo, SIGNAL(toggled(bool)), this, SLOT(chkShowInfoChanged(bool)));
    connect(chkShowSettings, SIGNAL(toggled(bool)), this, SLOT(chkShowSettingsChanged(bool)));
    connect(chkShowTerrain, SIGNAL(toggled(bool)), this, SLOT(chkShowTerrainChanged(bool)));
    connect(spnGimbalAngle, SIGNAL(valueChanged(double)), this, SLOT(spnGimbalAngleChanged(double)));
    connect(spnBand, SIGNAL(valueChanged(int)), this, SLOT(spnBandChanged(int)));
    connect(spnSweepAngle, SIGNAL(valueChanged(double)), this, SLOT(spnSweepAngleChanged(double)));
    connect(spnSweepFrequency, SIGNAL(valueChanged(double)), this, SLOT(spnSweepFrequencyChanged(double)));
    connect(spnPulseFrequency, SIGNAL(valueChanged(double)), this, SLOT(spnPulseFrequencyChanged(double)));
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
		std::string file = m_settings.value(K_TERRAIN_FILE, "").toString().toStdString();
		int band = m_settings.value(K_TERRAIN_BAND, 1).toInt();
		m_sim->setTerrainFile(file, band);

		Eigen::Vector3d go = m_sim->platform()->gimbal()->staticOrientation();
		go[1] = m_settings.value(K_GIMBAL_ANGLE, D_GIMBAL_ANGLE).toDouble() * PI / 180;
		m_sim->platform()->gimbal()->setStaticOrientation(go);
		uav::sim::SinGimbal* g = dynamic_cast<uav::sim::SinGimbal*>(m_sim->platform()->gimbal()); // Very inefficient.
		if(g) {
			g->setSweepFrequency(m_settings.value(K_SWEEP_FREQ, D_SWEEP_FREQ).toDouble());
			g->setSweepAngle(m_settings.value(K_SWEEP_ANGLE, D_SWEEP_ANGLE).toDouble() * PI / 180);
		}
		dynamic_cast<uav::sim::Rangefinder*>(m_sim->platform()->rangefinder())->setPulseFrequency(m_settings.value(K_PULSE_FREQ, D_PULSE_FREQ).toDouble());

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
	txtElevation->setText(QString::number(elev, 'f', 2));
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

void Sim1Viewer::spnPulseFrequencyChanged(double value) {
	m_settings.setValue(K_PULSE_FREQ, value);
	dynamic_cast<uav::sim::Rangefinder*>(m_sim->platform()->rangefinder())->setPulseFrequency(value);
}

void Sim1Viewer::spnSweepFrequencyChanged(double value) {
	m_settings.setValue(K_SWEEP_FREQ, value);
	uav::sim::SinGimbal* g = dynamic_cast<uav::sim::SinGimbal*>(m_sim->platform()->gimbal()); // Very inefficient.
	if(g)
		g->setSweepFrequency(value);
}

void Sim1Viewer::spnSweepAngleChanged(double value) {
	m_settings.setValue(K_SWEEP_ANGLE, value);
	uav::sim::SinGimbal* g = dynamic_cast<uav::sim::SinGimbal*>(m_sim->platform()->gimbal()); // Very inefficient.
	if(g)
		g->setSweepAngle(value * PI / 180);
}

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

void Sim1Viewer::btnResetClicked() {
	stop();
	m_settings.clear();
	applySettings();
}

void Sim1Viewer::btnTerrainFileClicked() {
	QString file = QFileDialog::getOpenFileName(m_form, "Choose a Terrain File", "", "*.tif");
	terrainFileChanged(file);
}

void Sim1Viewer::btnCloseFormClicked() {
	m_form->close();
}

void Sim1Viewer::spnGimbalAngleChanged(double value) {
	m_settings.setValue(K_GIMBAL_ANGLE, value);
	Eigen::Vector3d go = m_sim->platform()->gimbal()->staticOrientation();
	go[1] = value * PI / 180;
	m_sim->platform()->gimbal()->setStaticOrientation(go);
}

void Sim1Viewer::spnBandChanged(int value) {
	m_settings.setValue(K_TERRAIN_BAND, value);
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
	m_settings.setValue(K_EYEROT_Z, eyeRot[2]);
	m_settings.setValue(K_EYEDIST, glPanel->eyeDistance());
}

