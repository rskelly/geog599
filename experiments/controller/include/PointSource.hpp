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

template <class P>
class PointSource {
private:
	uav::PointFilter* m_filter;

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

	/**
	 * Reset the point reader to the beginning. Not all PointSources 
	 * will be able to do this.
	 */
	virtual void reset() {
		throw std::runtime_error("Reset not implemented.");
	}

	void setFilter(uav::PointFilter<P>* filter) {
		m_filter = filter;
	}

	virtual void computeBounds(double* bounds) {
		P pt;
		while(next(pt)) {
			if(pt.x() < bounds[0]) bounds[0] = pt.x();
			if(pt.x() > bounds[1]) bounds[1] = pt.x();
			if(pt.y() < bounds[2]) bounds[2] = pt.y();
			if(pt.y() > bounds[3]) bounds[3] = pt.y();
			if(pt.z() < bounds[4]) bounds[4] = pt.z();
			if(pt.z() > bounds[5]) bounds[5] = pt.z();
		}
		reset();
	}

	virtual ~PointSource() {
		if(m_filter)
			delete m_filter;
	}
};

} // uav



#endif /* INCLUDE_POINTSOURCE_HPP_ */
