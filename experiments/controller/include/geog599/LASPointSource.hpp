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
#include <unordered_set>

#include <liblas/liblas.hpp>

#include "ds/Octree.hpp"
#include "geog599/PointSource.hpp"

namespace uav {
namespace geog599 {

/**
 * Reads a stream of points from a LAS file and provides the PointSource
 * interface.
 */
template <class P>
class LASPointSource : public uav::geog599::PointSource<P> {
private:
	std::string m_filename;			///<! The filename of the las file.
	std::ifstream m_stream;			///<! The input stream for the las file.
	liblas::Reader* m_reader;		///<! The liblas point reader.
	uav::ds::Octree<P> m_tree;		///<! An Octree to store the points.
	std::list<P> m_filtered;		///<! A filtered list of points used for output.
	bool m_loaded;					///<! True if a las file is currently loaded.

	std::unordered_set<size_t> m_seen; ///<! Map to prevent duplicates. TODO: Hack.

public:

	/**
	 * Construct the LASPointSource using the given LAS file.
	 *
	 * @param filename A LAS file.
	 */
	LASPointSource(const std::string& filename) :
		LASPointSource() {
		load(filename);
	}

	/**
	 * Construct and empty LASPointSource.
	 */
	LASPointSource() :
		m_reader(nullptr),
		m_loaded(false) {}

	/**
	 * Load a point source file.
	 *
	 * @param filename A LAS file.
	 */
	void load(const std::string& filename) {
		m_seen.clear();
		m_loaded = false;
		m_filename = filename;
		m_tree.reset();
		m_stream.open(filename);
		if(m_reader)
			delete m_reader;
		m_reader = new liblas::Reader(m_stream);
		const liblas::Header& hdr = m_reader->GetHeader();
		const liblas::Bounds<double> bounds = hdr.GetExtent();
		m_tree.setBounds(bounds.minx(), bounds.maxx(), bounds.miny(), bounds.maxy(), bounds.minz(), bounds.maxz());
		while(m_reader->ReadNextPoint()) {
			const liblas::Point& lpt = m_reader->GetPoint();
			m_tree.add(Pt(lpt.GetX(), lpt.GetY(), lpt.GetZ(), lpt.GetTime()));
		}
		m_loaded = true;
	}

	/**
	 * Reload the LAS file.
	 */
	void reset() {
		if(m_loaded)
			load(m_filename);
	}

	/**
	 * Compute the bounds of the point cloud and write to the given six-element
	 * array (xmin, xmax, ymin, ymax, zmin, zmax).
	 *
	 * @param bounds A six-element list to contain the point cloud bounds.
	 */
	void computeBounds(double* bounds) {
		if(!m_loaded)
			throw std::runtime_error("Point source is not loaded. Call load first.");
		m_tree.getBounds(bounds);
	}

	/**
	 * Return a reference to the Octree used by this class.
	 *
	 * @param The Octree.
	 */
	const uav::ds::Octree<P>& octree() const {
		return m_tree;
	}

	bool next(P& pt) {
		if(m_filtered.empty()) {
			if(!this->m_filter)
				return m_tree.next(pt);
			this->m_filter->filter(m_filtered);
		}
		while(!m_filtered.empty()) {
			pt = m_filtered.front();
			m_filtered.pop_front();
			if(m_seen.find(pt.time()) == m_seen.end()) {
				m_seen.insert(pt.time());
				return true;
			}
		}
		return false;
	}

	/**
	 * Destroy the LASPointSource.
	 */
	~LASPointSource() {
		if(m_reader)
			delete m_reader;
	}

};

} // geog599
} // uav

#endif /* INCLUDE_SIM_LASPOINTSOURCE_HPP_ */
