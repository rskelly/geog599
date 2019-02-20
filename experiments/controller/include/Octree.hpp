/*
 * Octree.hpp
 *
 *  Created on: Feb 8, 2019
 *      Author: rob
 */

#ifndef INCLUDE_OCTREE_HPP_
#define INCLUDE_OCTREE_HPP_

#include <list>
#include <iostream>
#include <iomanip>

#include <Eigen/Geometry>

#define SIZE_LIMIT 100

namespace uav {
namespace ds {

template <class P>
class Node {
protected:
	Node<P>* nodes[8] = {0};
	std::list<P> items;
	bool isSplit;
	bool isLeaf;
	double bounds[6];

	Node* getNode(const P& item) {
		int idx = 0;
		if(item.x() > midx())
			idx |= 1;
		if(item.y() > midy())
			idx |= 2;
		if(item.z() > midz())
			idx |= 4;
		if(!nodes[idx]) {
			switch(idx) {
			case 0:
				nodes[0] = new Node(bounds[0], midx(), bounds[2], midy(), bounds[4], midz());
				break;
			case 1:
				nodes[1] = new Node(midx(), bounds[1], bounds[2], midy(), bounds[4], midz());
				break;
			case 2:
				nodes[2] = new Node(bounds[0], midx(), midy(), bounds[3], bounds[4], midz());
				break;
			case 3:
				nodes[3] = new Node(midx(), bounds[1], midy(), bounds[3], bounds[4], midz());
				break;
			case 4:
				nodes[4] = new Node(bounds[0], midx(), bounds[2], midy(), midz(), bounds[5]);
				break;
			case 5:
				nodes[5] = new Node(midx(), bounds[1], bounds[2], midy(), midz(), bounds[5]);
				break;
			case 6:
				nodes[6] = new Node(bounds[0], midx(), midy(), bounds[3], midz(), bounds[5]);
				break;
			case 7:
				nodes[7] = new Node(midx(), bounds[1], midy(), bounds[3], midz(), bounds[5]);
				break;
			default:
				throw std::runtime_error("Invalid index: " + idx);
			}
		}
		return nodes[idx];
	}

public:
	Node(double minx, double maxx, double miny, double maxy, double minz, double maxz) :
		isSplit(false),
		isLeaf(false),
		bounds({minx, maxx, miny, maxy, minz, maxz}) {}

	void reset() {
		for(int i = 0; i < 8; ++i) {
			if(nodes[i])
				delete nodes[i];
		}
	}

	double midx() const {
		return bounds[0] + (bounds[1] - bounds[0]) / 2.0;
	}

	double midy() const {
		return bounds[2] + (bounds[3] - bounds[2]) / 2.0;
	}

	double midz() const {
		return bounds[4] + (bounds[5] - bounds[4]) / 2.0;
	}

	double width() const {
		return bounds[1] - bounds[0];
	}

	double length() const {
		return bounds[3] - bounds[2];
	}

	double height() const {
		return bounds[5] - bounds[4];
	}

	void add(const P& item) {
		if(isLeaf || (!isLeaf && items.size() < SIZE_LIMIT)) {
			items.push_back(item);
		} else if(isSplit) {
			getNode(item)->add(item);
		} else {
			split();
			add(item);
		}
	}

	void remove(const P& item) {
		if(isSplit) {
			getNode(item)->remove(item);
		} else {
			for(const P& i : items) {
				if(i == item)
					items.erase(i);
			}
		}
	}

	void split() {
		if(!isSplit) {
			if((width() * height() * length()) <= 1) {
				isLeaf = true;
			} else {
				isSplit = true;
				for(const P& i : items)
					getNode(i)->add(i);
				items.clear();
			}
		}
	}

	/**
	 * Returns true if the centroid of the box is within the radius of the box plus max
	 * distance from the plane.
	 *
	 * @param plane The plane.
	 * @param maxDist The maximum distance from the plane.
	 */
	inline bool isNearPlane(const Eigen::Hyperplane<double, 3>& plane, double maxDist) const {
		Eigen::Vector3d v(midx(), midy(), midz());
		double r1 = std::pow(std::abs(bounds[0] - midx()) + maxDist, 2.0) + std::pow(std::abs(bounds[2] - midy()) + maxDist, 2.0) + std::pow(std::abs(bounds[4] - midz()) + maxDist, 2.0);
		double r2 = std::pow(plane.absDistance(v), 2.0);
		return  r2 <= r1;
	}

	size_t planeSearch(const Eigen::Hyperplane<double, 3>& plane, double maxDist, std::list<P>& result) const {

		//printBounds();

		if(!isNearPlane(plane, maxDist))
			return 0;

		size_t c = 0;

		if(isSplit) {

			for(int i = 0; i < 8; ++i) {
				if(nodes[i])
					c += nodes[i]->planeSearch(plane, maxDist, result);
			}

		} else {

			for(const P& i : items) {
				Eigen::Vector3d v(i.x(), i.y(), i.z());
				double d = plane.absDistance(v);
				//std::cerr << "Dist: " << d << ", " << maxDist << "\n";
				if(d <= maxDist) {
					result.push_back(i);
					++c;
				}
			}

		}

		return c;

	}

	void setBounds(double minx, double maxx, double miny, double maxy, double minz, double maxz) {
		double b[6] = {minx, maxx, miny, maxy, minz, maxz};
		for(int i = 0; i < 6; ++i)
			bounds[i] = b[i];
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

	void printBounds(int depth = 0) const {
		std::cerr << std::setprecision(9);
		std::stringstream ss;
		for(int i = 0; i < depth * 2; ++i)
			ss << " ";
		std::cerr << ss.str() << "x: " << bounds[0] << ", " << bounds[1] << "\n";
		std::cerr << ss.str() << "y: " << bounds[2] << ", " << bounds[3] << "\n";
		std::cerr << ss.str() << "z: " << bounds[4] << ", " << bounds[5] << "\n";
		for(int i = 0; i < 8; ++i) {
			if(nodes[i]) {
				std::cerr << ss.str() << i << ": -------------------------------------\n";
				nodes[i]->printBounds(depth + 1);
				std::cerr << ss.str() << "---------------------------------\n";
			}
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
