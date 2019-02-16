/*
 * Octree.hpp
 *
 *  Created on: Feb 8, 2019
 *      Author: rob
 */

#ifndef INCLUDE_OCTREE_HPP_
#define INCLUDE_OCTREE_HPP_

#include <list>

#include <Eigen/Geometry>

#define SIZE_LIMIT 100

namespace uav {
namespace ds {

template <class P>
class Node {
protected:
	Node<P>* nodes[8];
	std::list<P*> items;
	bool isSplit;
	double bounds[6];

public:
	Node(double minx, double maxx, double miny, double maxy, double minz, double maxz) :
		isSplit(false),
		bounds({minx, maxx, miny, maxy, minz, maxz}) {}

	void reset() {
		for(int i = 0; i < 8; ++i) {
			if(nodes[i])
				delete nodes[i];
		}
	}

	double midx() const {
		return (bounds[1] - bounds[0]) / 2.0;
	}

	double midy() const {
		return (bounds[3] - bounds[2]) / 2.0;
	}

	double midz() const {
		return (bounds[5] - bounds[4]) / 2.0;
	}

	void add(P* item) {
		if(isSplit) {
			int idx = 0;
			if(item->x() > midx())
				idx |= 1;
			if(item->y() > midy())
				idx |= 2;
			if(item->z() > midz())
				idx |= 4;
			nodes[idx]->add(item);
		} else if(items.size() >= SIZE_LIMIT) {
			split();
			add(item);
		} else {
			items.push_back(item);
		}
	}

	void remove(P* item) {
		if(isSplit) {
			int idx = 0;
			if(item->x() > midx())
				idx |= 1;
			if(item->y() > midy())
				idx |= 2;
			if(item->z() > midz())
				idx |= 4;
			nodes[idx]->remove(item);
		} else {
			for(P* i : items) {
				if(i == item)
					items.erase(i);
			}
		}
	}

	void split() {
		if(!isSplit) {
			nodes[0] = new Node(bounds[0], midx(), bounds[2], midy(), bounds[4], midz());
			nodes[1] = new Node(midx(), bounds[1], bounds[2], midy(), bounds[4], midz());
			nodes[2] = new Node(bounds[0], midx(), midy(), bounds[3], bounds[4], midz());
			nodes[3] = new Node(midx(), bounds[1], midy(), bounds[3], bounds[4], midz());
			nodes[0] = new Node(bounds[0], midx(), bounds[2], midy(), midz(), bounds[5]);
			nodes[1] = new Node(midx(), bounds[1], bounds[2], midy(), midz(), bounds[5]);
			nodes[2] = new Node(bounds[0], midx(), midy(), bounds[3], midz(), bounds[5]);
			nodes[3] = new Node(midx(), bounds[1], midy(), bounds[3], midz(), bounds[5]);
			isSplit = true;
			for(P* i : items)
				add(i);
			items.clear();
		}
	}

	inline bool isNearPlane(Eigen::Hyperplane& plane, double maxDist) {
		Eigen::Vector3d v(midx(), midy(), midz());
		double r = std::pow(bounds[0] - midx(), 2.0) + std::pow(bounds[2] - midy(), 2.0) + std::pow(bounds[4] - midz(), 2.0);
		return std::pow(plane.absDistance(v), 2.0) <= r + maxDist * maxDist;
	}

	size_t planeSearch(Eigen::Hyperplane& plane, double maxDist, std::list<P*>& result) {

		if(!isNearPlane(plane, maxDist))
			return 0;

		size_t c = 0;

		if(isSplit) {

			for(int i = 0; i < 8; ++i)
				c += nodes[i]->planeSearch(plane, maxDist, result);

		} else {

			for(P* i : items) {
				Eigen::Vector3d v(i->x(), i->y(), i->z());
				if(plane.absDistance(v) <= maxDist) {
					result.push_back(i);
					++c;
				}
			}

		}

		return c;

	}

	void setBounds(double minx, double maxx, double miny, double maxy, double minz, double maxz) {
		bounds = {minx, maxx, miny, maxy, minz, maxz};
	}

	void getBounds(double* bounds) {
		for(int i = 0; i < 6; ++i)
			bounds[i] = this->bounds[i];
	}
	
	~Node() {
		if(isSplit) {
			for(int i = 0; i < 6; ++i)
				delete nodes[i];
		}
	}
};

template <class P>
class Octree : public Node<P> {
public:
	Octree() : Octree(std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
				std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
				std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()) {}

	Octree(double minx, double maxx, double miny, double maxy, double minz, double maxz) :
			Node<P>(minx, maxx, miny, maxy, minz, maxz) {}

};


} // ds
} // uav



#endif /* INCLUDE_OCTREE_HPP_ */
