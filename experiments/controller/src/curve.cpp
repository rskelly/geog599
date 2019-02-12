/*
 * curve.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: rob
 */

#include <random>
#include <iostream>

#include "math/SmoothSpline.hpp"

class Pt {
public:
	double _x, _y;
	Pt(double x, double y) :
		_x(x), _y(y) {}
	double x() const {
		return _x;
	}
	double y() const {
		return _y;
	}
};

int main(int argc, char** argv) {

	std::default_random_engine generator;
	std::uniform_int_distribution<int> distribution(-100, 100);

	uav::math::SmoothSpline<Pt> spline;

	std::vector<Pt> pts;
	for(int i = 0; i < 20; ++i)
		pts.emplace_back((double) (i * 10), distribution(generator) / 10.0);

	std::vector<uav::math::SplinePiece> out;

	try {
		spline.fit(pts, 1, 5., 0.5, out);
		for(size_t i = 0; i < pts.size(); ++i) {
			const uav::math::SplinePiece& sp = out[i];
			std::cerr << pts[i].x() << "," << pts[i].y() << "," << sp.x0 << "," << sp.y0 << "," << sp.d0 << "," << sp.x1 << "," << sp.y1 << "," << sp.d1 << "\n";
		}
	} catch(const std::exception& ex) {
		std::cerr << ex.what() << "\n";
	}
}



