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

#include <Eigen/Core>

#include "platform.hpp"
#include "surface.hpp"
#include "sim/terrain.hpp"

namespace uav {
namespace viewer {

class RenderWidget : public QOpenGLWidget, protected QOpenGLFunctions {
private:
	bool m_initialized;
	double m_fov; // Field of view.
	int m_width, m_height; // Widget dimensions.
	int m_mouseX, m_mouseY; // Position of mouse on mouse button press.
	double m_rotX, m_rotY; // Proportion of angle to rotate eye vector (-1 - 1).
	double m_eyeDist; // Distance of eye from origin.
	uav::sim::Terrain* m_terrain;
	uav::Platform* m_platform;
	uav::surface::Surface* m_surface;
	Eigen::Vector3d m_eyePos;  			// The eye position, represented as a vector from the origin. Must be reversed to use for glLookAt

	void renderTerrain();
	void renderPlatform();
	void renderLaser();
	void renderSurface();
	void rotate(double x, double y);
	void zoom(double z);
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
