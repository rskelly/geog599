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
#include <QtCore/QSettings>

#include "ui_sim1viewer.h"

#include "sim/simulator.hpp"

namespace uav {
namespace viewer {

class Sim1Viewer : public QDialog, public Ui::Sim1Viewer, public uav::sim::SimulatorObserver {
	Q_OBJECT
private:
	double m_lastUpdate; 		// The last update time.
	uav::sim::Simulator* m_sim;
	QDialog* m_form;
	QSettings m_settings;

public:
	Sim1Viewer();
	void showForm();
	void setupUi(QDialog *Sim1Viewer);
	void setSimulator(uav::sim::Simulator& sim);
	void simUpdate(uav::sim::Simulator& sim);
	void updateInfo();
	~Sim1Viewer();

public slots:
	void chkShowInfoChanged(bool checked);
	void chkShowSettingsChanged(bool checked);
	void chkShowTerrainChanged(bool checked);
	void terrainFileChanged(QString file);
	void btnTerrainFileClicked();
	void btnCloseFormClicked();
	void btnStopClicked();
	void btnStartClicked();
	void spnGimbalAngleChanged(double value);
};


} // viewer
} // uav



#endif /* SRC_VIEWER_SIM1VIEWER_HPP_ */
