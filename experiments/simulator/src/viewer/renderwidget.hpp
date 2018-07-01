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

struct RenderParams {
	double minz;
	double maxz;
	double width;
	double height;
	double ratio;
	const double* trans;
};

class RenderWidget : public QOpenGLWidget, protected QOpenGLFunctions {
private:
	bool m_initialized;
	bool m_showTerrain;					// Whether terrain should be rendered.
	bool m_altDown;						// True if the alt key is pressed.
	int m_width, m_height; 				// Widget dimensions.
	int m_mouseX, m_mouseY; 			// Position of mouse on mouse button press.
	double m_eyeDist; 					// Distance of eye from origin.
	uav::sim::Terrain* m_terrain;		// Pointer to the terrain object. (Not owned.)
	uav::Platform* m_platform;			// Pointer to the platform object. (Not owned.)
	uav::surface::Surface* m_surface;	// Pointer to the surface object. (Not owned.)
	Eigen::Vector3d m_eyePos;  			// The eye position, represented as a vector from the origin. Must be reversed to use for glLookAt
	Eigen::Vector3d m_eyeRot; 			// Proportion of angle to rotate eye vector (-1 - 1).
	Eigen::Vector3d m_vOrigin;			// Origin of view.

	/**
	 * Called on paint to render the terrain.
	 */
	void renderTerrain(const RenderParams& params);

	/**
	 * Called on paint to render the platform.
	 */
	void renderPlatform(const RenderParams& params);

	/**
	 * Called on paint to render the laser beam.
	 */
	void renderLaser(const RenderParams& params);

	/**
	 * Called on paint to render the reconstructed surface.
	 */
	void renderSurface(const RenderParams& params);

	/**
	 * Rotate the view.
	 *
	 * @param dx Rotate in x (around the z axis).
	 * @param dy Rotate in y (around the x axis).
	 */
	void rotate(double dx, double dy);

	/**
	 * Translate the origin of the view by the specified amount.
	 *
	 * @param dx Translate in x.
	 * @param dy Translate in y.
	 */
	void translate(double dx, double dy);

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
	void keyPressEvent(QKeyEvent* evt);
	void keyReleaseEvent(QKeyEvent* evt);
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

	/**
	 * Set the eye position vector.
	 *
	 * @param pos The eye position vector.
	 */
	void setEyePosition(const Eigen::Vector3d& pos);

	/**
	 * Get the eye position vector.
	 *
	 * @return Get the eye position vector.
	 */
	const Eigen::Vector3d& eyePosition() const;

	/**
	 * Set the eye rotation vector.
	 *
	 * @param pos The eye rotation vector.
	 */
	void setEyeRotation(const Eigen::Vector3d& rot);

	/**
	 * Get the eye rotation vector.
	 *
	 * @return Get the eye rotation vector.
	 */
	const Eigen::Vector3d& eyeRotation() const;

	/**
	 * Set the eye origin vector.
	 *
	 * @param origin The origin vector.
	 */
	void setOrigin(const Eigen::Vector3d& origin);

	/**
	 * Get the origin vector.
	 *
	 * @return Get the origin vector.
	 */
	const Eigen::Vector3d& origin() const;

	/**
	 * Set the eye distance.
	 *
	 * @param dist The eye distance.
	 */
	void setEyeDistance(double dist);

	/**
	 * Get the eye distance.
	 *
	 * @return The eye distance.
	 */
	double eyeDistance() const;

};

} // viewer
} // uav

#endif /* SRC_VIEWER_RENDERWIDGET_HPP_ */
