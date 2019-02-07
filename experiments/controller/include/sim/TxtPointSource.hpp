/*
 * TxtPointSource.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_SIM_TXTPOINTSOURCE_HPP_
#define INCLUDE_SIM_TXTPOINTSOURCE_HPP_

#include <string>

#include <PointSource.hpp>

namespace uav {
namespace sim {

/**
 * Reads a stream of points (2d; y-z) from a text file and provides the PointSource
 * interface.
 */
template <class P>
class TxtPointSource : public uav::PointSource<P> {
public:

	/**
	 * Construct the TxtPointSource using the given comma-delimited source file.
	 *
	 * @param filename A comma-delimited file containing the y,z coordinates of each point. No header.
	 */
	TxtPointSource(const std::string& filename) {
		load(filename);
	}

	/**
	 * Construct and empty TxtPointSource.
	 */
	TxtPointSource();

	/**
	 * Load a point source file.
	 *
	 * @param filename A comma-delimited file containing the y,z coordinates of each point. No header.
	 */
	void load(const std::string& filename) {

	}

	bool next(P& pt) {

	}

	~TxtPointSource() {}

};

} // sim
} // uav

#endif /* INCLUDE_SIM_TXTPOINTSOURCE_HPP_ */
