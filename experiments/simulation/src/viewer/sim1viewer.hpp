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

/**
 * A Simulation viewer.
 */
class Sim1Viewer : public QDialog, public Ui::Sim1Viewer {
	Q_OBJECT
private:
	QSettings m_settings;
	std::thread m_thread;
	bool m_running;
	double m_lastUpdate; 		// The last update time.
	uav::sim::Simulator* m_sim;
	QDialog* m_form;

public:
	Sim1Viewer();

	/**
	 * Make the form visible,
	 */
	void showForm();

	void setupUi(QDialog *Sim1Viewer);

	/**
	 * Add the Simulator instance to the app.
	 *
	 * @param sim A Simulator instance.
	 */
	void setSimulator(uav::sim::Simulator& sim);

	/**
	 * Repaint the view.
	 */
	void update();

	/**
	 * Update the information view.
	 */
	void updateInfo();

	/**
	 * Applies the saved settings to the UI.
	 */
	void applySettings();

	/**
	 * Start the simulation and rendering.
	 */
	void start();

	/**
	 * Stop the simulation and rendering.
	 */
	void stop();

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
	void btnResetClicked();
	void spnGimbalAngleChanged(double value);
	void spnBandChanged(int value);
	void spnSweepAngleChanged(double value);
	void spnSweepFrequencyChanged(double value);
	void spnPulseFrequencyChanged(double value);
};


} // viewer
} // uav



#endif /* SRC_VIEWER_SIM1VIEWER_HPP_ */
