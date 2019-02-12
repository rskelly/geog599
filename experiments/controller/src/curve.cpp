/*
 * curve.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: rob
 */

#include <fstream>
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

void loadPoints(std::vector<Pt>& pts) {
	std::ifstream input("/home/rob/Documents/git/msc/docs/geog599_proposal/scripts/hull.csv");
	std::string x, y;
	while(std::getline(input, x, ',')) {
		std::getline(input, y);
		pts.emplace_back(atof(x.c_str()), atof(y.c_str()));
	}
}

int main(int argc, char** argv) {

	uav::math::SmoothSpline<Pt> spline;

	std::vector<Pt> pts;
	loadPoints(pts);

	try {
		spline.fit(pts, 0.1, 0.5);

		std::vector<double> x(pts.size());
		std::vector<double> y(pts.size());

		for(size_t i = 0; i < pts.size(); ++i)
			x[i] = pts[i].x();

		spline.evaluate(x, y, 0);

		for(size_t i = 0; i < pts.size(); ++i)
			std::cout << pts[i].x() << "," << pts[i].y() << "," << y[i] << "\n";

	} catch(const std::exception& ex) {
		std::cerr << ex.what() << "\n";
	}
}



