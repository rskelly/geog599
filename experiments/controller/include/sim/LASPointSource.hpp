/*
 * LASPointSource.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_SIM_LASPOINTSOURCE_HPP_
#define INCLUDE_SIM_LASPOINTSOURCE_HPP_

#include <string>
#include <fstream>

#include <liblas/liblas.hpp>

#include "Octree.hpp"
#include "PointSource.hpp"

namespace uav {
namespace sim {

/**
 * Reads a stream of points (2d; y-z) from a text file and provides the PointSource
 * interface.
 */
template <class P>
class LASPointSource : public uav::PointSource<P> {
private:
	liblas::Reader m_reader;
	std::ifstream m_stream;
	uav::ds::Octree<P> m_tree;
	std::list<P*> m_filtered;

public:

	/**
	 * Construct the LASPointSource using the given LAS file.
	 *
	 * @param filename A LAS file.
	 */
	LASPointSource(const std::string& filename) {
		load(filename);
	}

	/**
	 * Construct and empty LASPointSource.
	 */
	LASPointSource();

	/**
	 * Load a point source file.
	 *
	 * @param filename A comma-delimited file containing the y,z coordinates of each point. No header.
	 */
	void load(const std::string& filename) {
		m_stream.open(filename);
		m_reader.Reader(m_stream);
		const liblas::Header& hdr = m_reader.GetHeader();
		const liblas::Bounds<double> bounds = hdr.GetExtent();
		m_tree.setBounds(bounds.minx(), bounds.maxx(), bounds.miny(), bounds.maxy(), bounds.minz(), bounds.maxz());
		if(m_reader.ReadNextPoint()) {
			const liblas::Point& lpt = m_reader.GetPoint();
			m_tree.add(new Pt(lpt.GetX(), lpt.GetY(), lpt.GetZ(), lpt.GetTime()));
		}
	}

	const uav::ds::Octree<P>& octree() const {
		return m_tree;
	}

	bool next(P& pt) {
		if(m_filtered.empty()) {
			if(!m_filter)
				return false;
			m_filter->filter(m_filtered);
		}
		if(!m_filtered.empty()) {
			Pt* p = m_filtered.front();
			pt.x(p->x());
			pt.y(p->y());
			pt.z(p->z());
			pt.time(p->time());
			m_filtered.pop_front();
			return true;
		}
		return false;
	}

	~LASPointSource() {
	}

};

} // sim
} // uav

#endif /* INCLUDE_SIM_LASPOINTSOURCE_HPP_ */
