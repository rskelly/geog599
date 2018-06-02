/*
 * sim1viewer.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

#include <QtWidgets/QFileDialog>
#include <QtCore/QString>
#include <QtCore/QSettings>

#include "sim1viewer.hpp"

#define K_TERRAIN_FILE "terrainFile"

using namespace uav::viewer;

Sim1Viewer::Sim1Viewer() :
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
}

void Sim1Viewer::showForm() {
	if(!m_form)
		m_form = new QDialog();
	this->setupUi(m_form);
	m_form->show();
}

std::string Sim1Viewer::terrainFile() const {
	return m_settings.find(K_TERRAIN_FILE)->second;
}

// slots

void Sim1Viewer::terrainFileChanged(QString file) {
	m_settings[K_TERRAIN_FILE] = file.toStdString();
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

