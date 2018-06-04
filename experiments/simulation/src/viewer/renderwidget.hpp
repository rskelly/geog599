/*
 * renderwidget.hpp
 *
 *  Created on: Jun 2, 2018
 *      Author: rob
 */

#ifndef SRC_VIEWER_RENDERWIDGET_HPP_
#define SRC_VIEWER_RENDERWIDGET_HPP_

#include <QtWidgets/QOpenGLWidget>
#include <QtGui/QOpenGLFunctions>

#include "sim/geometry.hpp"

namespace uav {
namespace viewer {

class RenderWidget : public QOpenGLWidget, protected QOpenGLFunctions {
private:
	std::vector<std::vector<double> > m_triangulations;
	double m_transform[6];
	double m_width;
	double m_height;
	double m_minz;
	double m_maxz;
	void renderTriangulations();
protected:
	void resizeGL(int w, int h);
	void paintGL();
	void initializeGL();
public:
	RenderWidget(QWidget* parent);
	void addTriangulation(const std::vector<double>& tri);
	void setTransform(const double* trans);
	void setSize(double width, double height);
	void setZRange(double minz, double maxz);
};

} // viewer
} // uav

#endif /* SRC_VIEWER_RENDERWIDGET_HPP_ */
