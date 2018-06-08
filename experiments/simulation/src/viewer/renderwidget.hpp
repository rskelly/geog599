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

#include "platform.hpp"
#include "surface.hpp"
#include "sim/terrain.hpp"

namespace uav {
namespace viewer {

class RenderWidget : public QOpenGLWidget, protected QOpenGLFunctions {
private:
	bool m_initialized;
	uav::sim::Terrain* m_terrain;
	uav::Platform* m_platform;
	uav::surface::Surface* m_surface;
	void renderTerrain();
	void renderPlatform();
	void renderLaser();
	void renderSurface();
	void rotate(int x, int y);
protected:
	void resizeGL(int w, int h);
	void paintGL();
	void initializeGL();
	bool event(QEvent* evt);
public:
	RenderWidget(QWidget* parent);
	void setTerrain(uav::sim::Terrain* terrain);
	void setPlatform(uav::Platform* platform);
	void setSurface(uav::surface::Surface* surface);
};

} // viewer
} // uav

#endif /* SRC_VIEWER_RENDERWIDGET_HPP_ */
