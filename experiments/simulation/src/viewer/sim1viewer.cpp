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
#include <QtCore/QSettings>

#include "viewer/sim1viewer.hpp"
#include "viewer/renderwidget.hpp"
#include "geometry.hpp"
#include "rangefinder.hpp"
#include "platform.hpp"
#include "util.hpp"

#define K_TERRAIN_FILE "terrainFile"
#define K_SHOW_TERRAIN "showTerrain"

using namespace uav::viewer;
using namespace uav::sim;
using namespace uav::util;


Sim1Viewer::Sim1Viewer() :
		m_lastUpdate(0),
		m_sim(nullptr),
		m_form(nullptr) {

	// Load the settings from the local store.
	QSettings settings("sim1", "dijital.ca");
	m_settings[K_TERRAIN_FILE] = settings.value(K_TERRAIN_FILE, "").toString().toStdString();
	m_settings[K_SHOW_TERRAIN] = settings.value(K_SHOW_TERRAIN, "true").toString().toStdString();
}

void Sim1Viewer::setupUi(QDialog *Sim1Viewer) {
	Ui::Sim1Viewer::setupUi(Sim1Viewer);

	// Set the terrain file on the text box from the settings map.
	txtTerrainFile->setText(QString::fromStdString(m_settings[K_TERRAIN_FILE]));
	// Show whether terrain will be rendered.
	chkShowTerrain->setChecked(m_settings[K_SHOW_TERRAIN] == "true");

	// Connect events.
	connect(txtTerrainFile, SIGNAL(textEdited(QString)), this, SLOT(terrainFileChanged(QString)));
    connect(btnTerrainFile, SIGNAL(clicked()), this, SLOT(btnTerrainFileClicked()));
    connect(btnClose, SIGNAL(clicked()), this, SLOT(btnCloseFormClicked()));
    connect(btnStart, SIGNAL(clicked()), this, SLOT(btnStartClicked()));
    connect(btnStop, SIGNAL(clicked()), this, SLOT(btnStopClicked()));
    connect(chkShowInfo, SIGNAL(toggled(bool)), SLOT(chkShowInfoChanged(bool)));
    connect(chkShowSettings, SIGNAL(toggled(bool)), SLOT(chkShowSettingsChanged(bool)));
    connect(chkShowTerrain, SIGNAL(toggled(bool)), SLOT(chkShowTerrainChanged(bool)));
}

void Sim1Viewer::setSimulator(Simulator& sim) {
	m_sim = &sim;
	m_sim->addObserver(this);
}

void Sim1Viewer::updateInfo() {
	double elev = m_sim->platform()->platformState().altitude();
	QString e = QString::number(elev, 'f', 2);
	txtElevation->setText(e);

}

void Sim1Viewer::simUpdate(Simulator& sim) {
	double t = uavtime();
	if(t - m_lastUpdate > 0.05) {
		glPanel->update();
		updateInfo();
		m_lastUpdate = t;
	}
}

void Sim1Viewer::showForm() {
	if(!m_form)
		m_form = new QDialog();
	this->setupUi(m_form);
	m_form->show();
}

// slots

void Sim1Viewer::chkShowTerrainChanged(bool show) {
	glPanel->showTerrain(show);
	m_settings[K_SHOW_TERRAIN] = show ? "true" : "false";
}

void Sim1Viewer::chkShowInfoChanged(bool checked) {
	frmInfo->setVisible(checked);
}

void Sim1Viewer::chkShowSettingsChanged(bool checked) {
	frmSettings->setVisible(checked);
}

void Sim1Viewer::terrainFileChanged(QString file) {
	m_settings[K_TERRAIN_FILE] = file.toStdString();
	txtTerrainFile->setText(file);
}

void Sim1Viewer::btnStartClicked() {
	if(!m_sim)
		throw std::runtime_error("Simulator not set.");
	m_sim->setTerrainFile(m_settings[K_TERRAIN_FILE]);
	m_sim->addObserver(this);
	m_sim->start();

	glPanel->setTerrain(m_sim->terrain());
	glPanel->setPlatform(m_sim->platform());
	glPanel->setSurface(m_sim->platform()->surface());

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
		m_form = nullptr;
	}
}


Sim1Viewer::~Sim1Viewer() {
	// Save the settings to the store.
	QSettings settings("sim1", "dijital.ca");
	for(const auto& item : m_settings)
		settings.setValue(QString::fromStdString(item.first), QString::fromStdString(item.second));
	if(m_form)
		delete m_form;
}

