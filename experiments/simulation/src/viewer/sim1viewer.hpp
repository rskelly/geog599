/*
 * sim1viewer.hpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

#ifndef SRC_VIEWER_SIM1VIEWER_HPP_
#define SRC_VIEWER_SIM1VIEWER_HPP_

#include <string>

#include <QtWidgets/QDialog>

#include "ui_sim1viewer.h"

namespace uav {
namespace viewer {

class Sim1Viewer : public Ui::Sim1Viewer {
private:
	QDialog* m_form;
	std::string m_terrainFile;

public:
	Sim1Viewer();
	void showForm();
	virtual ~Sim1Viewer();

	const std::string& terrainFile() const;

public slots:
	void terrainFileChanged(QString file);
	void btnTerrainFileClicked();
	void closeFormClicked();
};


} // viewer
} // uav



#endif /* SRC_VIEWER_SIM1VIEWER_HPP_ */
