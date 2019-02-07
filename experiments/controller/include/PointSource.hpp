/*
 * PointSource.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_POINTSOURCE_HPP_
#define INCLUDE_POINTSOURCE_HPP_

namespace uav {

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
	virtual bool next(P& pt) = 0;

	virtual ~PointSource() {}
};

} // uav



#endif /* INCLUDE_POINTSOURCE_HPP_ */
