/*
 * sim1viewer.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

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

Sim1Viewer::~Sim1Viewer() {
	if(m_form)
		delete m_form;
}

