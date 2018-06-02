/*
 * sim1viewer.hpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

#ifndef SRC_VIEWER_SIM1VIEWER_HPP_
#define SRC_VIEWER_SIM1VIEWER_HPP_

#include <string>
#include <unordered_map>

#include <QtWidgets/QDialog>

#include "ui_sim1viewer.h"

#include "sim/simulator.hpp"

namespace uav {
namespace viewer {

class Sim1Viewer : public QDialog, public Ui::Sim1Viewer, public SimulatorObserver {
	Q_OBJECT
private:
	Simulator* m_sim;
	QDialog* m_form;
	std::unordered_map<std::string, std::string> m_settings;

public:
	Sim1Viewer();
	void showForm();
	void setupUi(QDialog *Sim1Viewer);
	//std::string terrainFile() const;
	void setSimulator(uav::sim::Simulator& sim);
	void simUpdate(uav::sim::Simulator& sim);
	virtual ~Sim1Viewer();

public slots:
	void terrainFileChanged(QString file);
	void btnTerrainFileClicked();
	void btnCloseFormClicked();
	void btnStopClicked();
	void btnStartClicked();
};


} // viewer
} // uav



#endif /* SRC_VIEWER_SIM1VIEWER_HPP_ */
