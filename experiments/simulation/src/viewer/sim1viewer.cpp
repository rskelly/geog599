/*
 * sim1viewer.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

#include <QtWidgets/QFileDialog>
#include <QtCore/QString>

#include "sim1viewer.hpp"

using namespace uav::viewer;

Sim1Viewer::Sim1Viewer() :
		m_form(nullptr) {}

void Sim1Viewer::showForm() {
	if(!m_form)
		m_form = new QDialog();
	this->setupUi(m_form);
	m_form->show();
}

const std::string& Sim1Viewer::terrainFile() const {
	return m_terrainFile;
}

// slots

void Sim1Viewer::terrainFileChanged(QString file) {
	m_terrainFile = file.toStdString();
}

void Sim1Viewer::btnTerrainFileClicked() {
	// TODO: Remember last dir.
	terrainFileChanged(QFileDialog::getOpenFileName(m_form, "Choose a Terrain File", "", "*.tif"));

}

void Sim1Viewer::closeFormClicked() {
	if(m_form) {
		m_form->close();
		delete m_form;
	}
}


Sim1Viewer::~Sim1Viewer() {
	if(m_form)
		delete m_form;
}

