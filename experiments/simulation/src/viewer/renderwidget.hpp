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

namespace uav {
namespace viewer {

class RenderWidget : public QOpenGLWidget, protected QOpenGLFunctions {
protected:
	void resizeGL(int w, int h);
	void paintGL();
	void initializeGL();
public:
	RenderWidget(QWidget* parent);
};

} // viewer
} // uav

#endif /* SRC_VIEWER_RENDERWIDGET_HPP_ */
