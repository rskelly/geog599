/*
 * sim1viewer.hpp
 *
 *  Created on: Jun 1, 2018
 *      Author: rob
 */

#ifndef SRC_VIEWER_SIM1VIEWER_HPP_
#define SRC_VIEWER_SIM1VIEWER_HPP_

#include <QtWidgets/QDialog>

#include "ui_sim1viewer.h"

namespace uav {
namespace viewer {

class Sim1Viewer : public Ui::Sim1Viewer {
private:
	QDialog* m_form;

public:
	Sim1Viewer();
	void showForm();
	~Sim1Viewer();
};


} // viewer
} // uav



#endif /* SRC_VIEWER_SIM1VIEWER_HPP_ */
