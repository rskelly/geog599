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
	bool m_showTerrain;					// Whether terrain should be rendered.
	double m_fov; 						// Field of view.
	int m_width, m_height; 				// Widget dimensions.
	int m_mouseX, m_mouseY; 			// Position of mouse on mouse button press.
	double m_rotX, m_rotY; 				// Proportion of angle to rotate eye vector (-1 - 1).
	double m_eyeDist; 					// Distance of eye from origin.
	uav::sim::Terrain* m_terrain;		// Pointer to the terrain object. (Not owned.)
	uav::Platform* m_platform;			// Pointer to the platform object. (Not owned.)
	uav::surface::Surface* m_surface;	// Pointer to the surface object. (Not owned.)
	Eigen::Vector3d m_eyePos;  			// The eye position, represented as a vector from the origin. Must be reversed to use for glLookAt

	/**
	 * Called on paint to render the terrain.
	 */
	void renderTerrain();

	/**
	 * Called on paint to render the platform.
	 */
	void renderPlatform();

	/**
	 * Called on paint to render the laser beam.
	 */
	void renderLaser();

	/**
	 * Called on paint to render the reconstructed surface.
	 */
	void renderSurface();

	/**
	 * Rotate the view.
	 *
	 * @param dx Rotate in x (around the z axis).
	 * @param dy Rotate in y (around the x axis).
	 */
	void rotate(double dx, double dy);

	/**
	 * Zoom by the specified amount. Adjusts the eye distance from the origin.
	 *
	 * @param z Amount by which to adjust the eye distance.
	 */
	void zoom(double z);

protected:
	void resizeGL(int w, int h);
	void paintGL();
	void initializeGL();
	bool event(QEvent* evt);

public:

	RenderWidget(QWidget* parent);

	/**
	 * Toggle the visibility of the terrain.
	 *
	 * @param show If true, terrain will be rendered.
	 */
	void showTerrain(bool show);

	/**
	 * Set a pointer to the terrain. Caller keeps ownership of the pointer.
	 *
	 * @param terrain A pointer to a terrain object.
	 */
	void setTerrain(uav::sim::Terrain* terrain);

	/**
	 * Set a pointer to the platform. Caller keeps ownership of the pointer.
	 *
	 * @param platform  A pointer to a platform object.
	 */
	void setPlatform(uav::Platform* platform);

	/**
	 * Set a pointer to the surface. Caller keeps ownership of the pointer.
	 *
	 * @param surface A pointer to a surface object.
	 */
	void setSurface(uav::surface::Surface* surface);
};

} // viewer
} // uav

#endif /* SRC_VIEWER_RENDERWIDGET_HPP_ */
