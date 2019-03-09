/*
 * profile.cpp
 *
 *  Created on: Mar 7, 2019
 *      Author: rob
 */

#include "profile.hpp"

ProfileDialog* __inst;

void ProfileDialog::setupUi(QDialog* dialog) {
	Ui::ProfileDialog::setupUi(dialog);
	if(__inst)
		throw std::runtime_error("An instance of this window is running.");
	__inst = this;
	connect(btnClose, SIGNAL(clicked()), this, SLOT(closeClicked()));
	canvas->update();
}

void ProfileDialog::addDrawConfig(DrawConfig* config) {
	canvas->configs.push_back(config);
}

void ProfileDialog::removeDrawConfig(DrawConfig* config) {
	//canvas->configs.erase(config);
}

void ProfileDialog::draw() {
	canvas->update();
}

void ProfileDialog::setBounds(double minx, double miny, double maxx, double maxy) {
	canvas->setBounds(minx, miny, maxx, maxy);
}

void ProfileDialog::closeClicked() {
	QApplication::instance()->quit();
}

ProfileDialog* ProfileDialog::instance() {
	return __inst;
}

