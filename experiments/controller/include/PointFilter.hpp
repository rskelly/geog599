/*
 * PointFilter.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_POINTFILTER_HPP_
#define INCLUDE_POINTFILTER_HPP_

#include <list>

namespace uav {

/**
 * Provides an interface for filtering unneeded points out of a point list.
 * PointFilter instances may be chained.
 */
template <class P>
class PointFilter {
private:
	PointFilter* m_nextFilter;

protected:

	/**
	 * Filter implementation.
	 */
	virtual void doFilter(std::list<P>& pts) = 0;

public:

	/**
	 * Add a filter that will be called on the point set after this filter's operations are complete.
	 *
	 * @param next A pointer to the next PointFilter.
	 */
	void setNextFilter(PointFilter<P>* next) {
		m_nextFilter = next;
	}

	/**
	 * Filter the point list.
	 *
	 * @param pts A list of points.
	 */
	void filter(std::list<P>& pts) {
		doFilter(pts);
		if(m_nextFilter)
			m_nextFilter->filter(pts);
	}

	virtual ~PointFilter() {}
};

} // uav



#endif /* INCLUDE_POINTFILTER_HPP_ */
