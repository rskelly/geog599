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

/**
 * Implementation of an octree node.
 */
template <class P>
class Node {
protected:
	Node<P>* nodes[8] = {0};	///<! A list of pointers to child nodes.
	std::vector<P> items;		///<! List of items contained in a leaf node.
	bool isSplit;				///<! True if the node is split. Will have at least one child.
	bool isLeaf;				///<! True if the node is a leaf.
	double bounds[6];			///<! The bounds of the node. {xmin, xmax, ymin, ymax, zmin, zmax}
	int iterIdx;				///<! Iteration index.
	bool needReset;				///<! True if the iteration index should be reset.

	/**
	 * Return a pointer to the node that would contain the given item, based
	 * on its extents. If the node does not exist, create it.
	 *
	 * @param item An item.
	 * @return A pointer to the node that would contain the item.
	 */
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

	double area() const {
		return width() * height() + length() * height() + width() * length();
	}

	/**
	 * Split the node and distribute its children to the child nodes.
	 */
	void split() {
		if(!isSplit) {
			if(area() <= 1) {
				isLeaf = true;
			} else {
				isSplit = true;
				for(const P& i : items)
					getNode(i)->add(i);
				items.clear();
			}
		}
	}


public:

	/**
	 * Construct a node with the given extents.
	 *
	 * @param minx The minimum x-coordinate.
	 * @param maxx Tha maximum x-coordinate.
	 * @param miny The minimum y-coordinate.
	 * @param maxy Tha maximum y-coordinate.
	 * @param minz The minimum z-coordinate.
	 * @param maxz Tha maximum z-coordinate.
	 */
	Node(double minx, double maxx, double miny, double maxy, double minz, double maxz) :
		isSplit(false),
		isLeaf(false),
		bounds({minx, maxx, miny, maxy, minz, maxz}),
		iterIdx(0),
		needReset(true) {}

	/**
	 * Delete the child nodes and reset the pointers to null.
	 */
	void clear() {
		reset();
		for(int i = 0; i < 8; ++i) {
			if(nodes[i]) {
				delete nodes[i];
				nodes[i] = nullptr;
			}
		}
	}

	/**
	 * The middle x-coordinate.
	 *
	 * @return The middle x-coordinate.
	 */
	double midx() const {
		return bounds[0] + (bounds[1] - bounds[0]) / 2.0;
	}

	/**
	 * The minimum x-coordinate.
	 *
	 * @return The minimum x-coordinate.
	 */
	double minx() const {
		return bounds[0];
	}

	/**
	 * The maximum x-coordinate.
	 *
	 * @return The maximum x-coordinate.
	 */
	double maxx() const {
		return bounds[1];
	}

	/**
	 * The middle y-coordinate.
	 *
	 * @return The middle y-coordinate.
	 */
	double midy() const {
		return bounds[2] + (bounds[3] - bounds[2]) / 2.0;
	}

	/**
	 * The minimum y-coordinate.
	 *
	 * @return The minimum y-coordinate.
	 */
	double miny() const {
		return bounds[2];
	}

	/**
	 * The maximum y-coordinate.
	 *
	 * @return The maximum y-coordinate.
	 */
	double maxy() const {
		return bounds[3];
	}

	/**
	 * The middle z-coordinate.
	 *
	 * @return The middle z-coordinate.
	 */
	double midz() const {
		return bounds[4] + (bounds[5] - bounds[4]) / 2.0;
	}

	/**
	 * The minimum z-coordinate.
	 *
	 * @return The minimum z-coordinate.
	 */
	double minz() const {
		return bounds[4];
	}

	/**
	 * The maximum z-coordinate.
	 *
	 * @return The maximum z-coordinate.
	 */
	double maxz() const {
		return bounds[5];
	}

	/**
	 * The width of the node's extent.
	 *				return true;
	 *
	 * @return The width of the node's extent.
	 */
	double width() const {
		return bounds[1] - bounds[0];
	}

	/**
	 * The length of the node's extent.
	 *
	 * @return The length of the node's extent.
	 */
	double length() const {
		return bounds[3] - bounds[2];
	}

	/**
	 * The height of the node's extent.
	 *
	 * @return The height of the node's extent.
	 */
	double height() const {
		return bounds[5] - bounds[4];
	}

	void reset() {
		iterIdx = 0;
		for(size_t i = 0; i < 8; ++i) {
			if(nodes[i]) nodes[i]->reset();
		}
		needReset = false;
	}

	bool next(P& p) {
		if(needReset)
			reset();
		if(isLeaf) {
			if(iterIdx >= items.size())
				return false;
			p = items[iterIdx++];
			return true;
		} else {
			if(iterIdx >= 8)
				return false;
			do {
				if(nodes[iterIdx] && nodes[iterIdx]->next(p))
					return true;
				++iterIdx;
			} while(iterIdx < 8);
			return false;
		}
	}
	/**
	 * Add an item to the node. If the item limit is exceeded, the node will split.
	 *
	 * @param item An item.
	 */
	void add(const P& item) {
		needReset = true;
		//if(isLeaf || (!isLeaf && items.size() < SIZE_LIMIT)) {
		isLeaf = true;
			items.push_back(item);
		//} else if(isSplit) {
		//	getNode(item)->add(item);
		//} else {
		//	split();
		//	add(item);
		//}
	}

	/**
	 * Remove an item from the node. If the number of items
	 * falls below the limit, the node will not un-split.
	 *
	 * @param item An item.
	 */
	void remove(const P& item) {
		needReset = true;
		if(isSplit) {
			getNode(item)->remove(item);
		} else {
			for(const P& i : items) {
				if(i == item)
					items.erase(i);
			}
		}
	}

	/**
	 * Return true if the node contains the given 2D coordinate.
	 *
	 * @param x The x-coordinate.
	 * @param y The y-coordinate.
	 * @return True if the node contains the given 2D coordinate.
	 */
	bool contains(double x, double y) const {
		return x >= minx() && x <= maxx() && y >= miny() && y <= maxy();
	}

	/**
	 * Return the item nearest the given 2D coordinate.
	 *
	 * @param x The x-coordinate.
	 * @param y The y-coordinate.
	 * @return The item nearest the given 2D coordinate.
	 */
	const P* nearest(double x, double y) const {
		if(isSplit) {
			double d, mind = std::numeric_limits<double>::max();
			const P* cur = nullptr;
			const P* pt = nullptr;
			for(int i = 0; i < 8; ++i) {
				if(nodes[i]) {
					pt = nodes[i]->nearest(x, y);
					if(pt) {
						if(!cur || (d = std::pow(pt->x() - x, 2.0) + std::pow(pt->y() - y, 2.0)) < mind) {
							cur = pt;
							mind = d;
						}
					}
				}
			}
			return cur;
		} else {
			double mind = std::numeric_limits<double>::max();
			const P* cur = nullptr;
			for(const P& i : items) {
				double d = std::pow(i.x() - x, 2.0) + std::pow(i.y() - y, 2.0);
				if(d < mind) {
					mind = d;
					cur = &i;
				}
			}
			return cur;
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
		double r1 = std::pow(std::abs(minx() - midx()) + maxDist, 2.0) + std::pow(std::abs(miny() - midy()) + maxDist, 2.0) + std::pow(std::abs(minz() - midz()) + maxDist, 2.0);
		double r2 = std::pow(plane.absDistance(v), 2.0);
		return  r2 <= r1;
	}

	/**
	 * Search the node for all items within the given distance of the plane. Populate the result list
	 * with the found items, and return their number.
	 *
	 * @param plane An Eigen::Hyperplane.
	 * @param maxDist The maximum distance of an item from the plane.
	 * @param result The result list.
	 * @return The number of items found.
	 */
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
				if(d <= maxDist) {
					result.push_back(i);
					++c;
				}
			}

		}

		return c;

	}

	/**
	 * Set the extents of the node.
	 *
	 * @param minx The minimum x-coordinate.
	 * @param maxx Tha maximum x-coordinate.
	 * @param miny The minimum y-coordinate.
	 * @param maxy Tha maximum y-coordinate.
	 * @param minz The minimum z-coordinate.
	 * @param maxz Tha maximum z-coordinate.
	 */
	void setBounds(double minx, double maxx, double miny, double maxy, double minz, double maxz) {
		double b[6] = {minx, maxx, miny, maxy, minz, maxz};
		setBounds(b);
	}

	void setBounds(double* b) {
		for(int i = 0; i < 6; ++i)
			bounds[i] = b[i];
	}

	/**
	 * Return the extents of the node.
	 *
	 * @param bounds An array to contain the extents. Must have room for six elements.
	 */
	void getBounds(double* bounds) {
		for(int i = 0; i < 6; ++i)
			bounds[i] = this->bounds[i];
	}
	
	/*
	void computeBounds() {
		if(split) {
			for(int i = 0; i < 8; ++i) {
				if(nodes[i])
					nodes[i]->computeBounds();
			}
		} else {
			bounds[0] = bounds[2] = bounds[4] = std::numeric_limits<double>::max();
			bounds[1] = bounds[3] = bounds[5] = std::numeric_limits<double>::lowest();
			for(const P& p : items) {
				if(p.x() < minx) minx = p.x();
				if(p.x() > maxx) maxx = p.x();
				if(p.y() < miny) miny = p.y();
				if(p.y() > maxy) maxy = p.y();
				if(p.z() < minz) minz = p.z();
				if(p.z() > maxz) maxz = p.z();
			}
		}
	}
	*/

	/**
	 * Destroy the node and its children.
	 */
	~Node() {
		if(isSplit) {
			for(int i = 0; i < 6; ++i)
				delete nodes[i];
		}
	}

	/**
	 * Print the extents of the node up to the given depth.
	 */
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

/**
 * A wrapper for the octree Node, providing default instantiations.
 */
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
