/*
 * TrajectoryPlanner.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_TRAJECTORYPLANNER_HPP_
#define INCLUDE_TRAJECTORYPLANNER_HPP_

#include "PointSorter.hpp"
#include "PointSource.hpp"
#include "PointFilter.hpp"

namespace uav {

class Pt {
public:
	double y;
	double z;
};

template <class P>
class TrajectoryPlanner {
private:
	uav::PointSource<P>* m_ptSource;
	uav::PointSorter<P>* m_ptSorter;
	uav::PointFilter<P>* m_ptFilter;

public:

	/**
	 * Set the source of points.
	 *
	 * @param psrc The PointSource.
	 */
	void setPointSource(uav::PointSource<P>* psrc) {
		m_ptSource = psrc;
	}

	/**
	 * Set the point sorter (optional).
	 *
	 * @param psort A PointSorter.
	 */
	void setPointSorter(uav::PointSorter<P>* psort) {
		m_ptSorter = psort;
	}

	/**
	 * Set the point filter (optional).
	 *
	 * @param pfilter A PointFilter.
	 */
	void setPointFilter(uav::PointFilter<P>* pfilter) {
		m_ptFilter = pfilter;
	}

	/**
	 * Generate the trajectories.
	 */
	void generate() {

	}
};

} // uav



#endif /* INCLUDE_TRAJECTORYPLANNER_HPP_ */
