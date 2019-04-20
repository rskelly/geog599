/*
 * PointSource.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_POINTSOURCE_HPP_
#define INCLUDE_POINTSOURCE_HPP_

#include "PointFilter.hpp"

namespace uav {
namespace geog599 {

/**
 * Provides a source of 3D Cartesian points from a point cloud or stream.
 */
template <class P>
class PointSource {
public:

	/**
	 * Return the next available point. If none is available, false is returned
	 * and the point is unchanged. Otherwise the point is updated and the method
	 * returns true.
	 *
	 * @param pt A point object.
	 * @return true if a new point was available.
	 */
	virtual int getPoints(std::list<P>& pts) = 0;

	/**
	 * Reset the point reader to the beginning. Not all PointSources 
	 * will be able to do this.
	 */
	virtual void reset() {
		throw std::runtime_error("Reset not implemented.");
	}

	/**
	 * Destroy the PointSource.
	 */
	virtual ~PointSource() {
	}

};

} // geog599
} // uav



#endif /* INCLUDE_POINTSOURCE_HPP_ */
