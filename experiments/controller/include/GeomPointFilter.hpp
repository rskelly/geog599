/*
 * GeomPointFilter.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_GEOMPOINTFILTER_HPP_
#define INCLUDE_GEOMPOINTFILTER_HPP_

#include "PointFilter.hpp"

namespace uav {

template <class P>
class GeomPointFilter : public PointFilter<P> {
private:
	double m_minY;

protected:

	void doFilter(std::list<P*>& pts) {
		std::list<P*> lst;
		for(P* pt : pts) {
			if(pt->y() >= m_minY)
				lst.push_back(pt);
		}
		pts.swap(lst);
	}

public:

	/**
	 * Set the minimum y coordinate. All points with y lower than this are removed.
	 *
	 * @param miny The minimum y coordinate.
	 */
	void setMinY(double miny) {
		m_minY = miny;
	}

};


} // uav



#endif /* INCLUDE_GEOMPOINTFILTER_HPP_ */
