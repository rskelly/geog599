/*
 * curve.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: rob
 */

#include <fstream>
#include <iostream>
#include <sstream>

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
	double z() const {
		return 0;
	}
	double operator[](int idx) const {
		switch(idx % 3) {
		case 0: return _x;
		case 1: return _y;
		default: return 0;
		}
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

	std::vector<double> s = {0.01, 0.1, 0.25, 0.5, 0.75, 1., 2., 5., 10.};
	std::vector<double> w = {0.01, 0.1, 0.25, 0.5, 0.75, 0.99, 1.};

	uav::math::SmoothSpline<Pt> spline;

	std::vector<Pt> pts;
	loadPoints(pts);

	for(size_t i = 0; i < s.size(); ++i) {
		for(size_t j = 0; j < w.size(); ++j) {
			double ww = w[j];
			double ss = s[i];
			std::stringstream ofile;
			ofile << "result_" << ss << "_" << ww << ".csv";
			std::ofstream output(ofile.str());
			try {
				spline.fit(pts, ww, ss);

				std::vector<double> x(pts.size());
				std::vector<double> y(pts.size());
				std::vector<double> y1(pts.size());
				std::vector<double> y2(pts.size());

				for(size_t i = 0; i < pts.size(); ++i)
					x[i] = pts[i].x();

				spline.evaluate(x, y, 0);
				spline.evaluate(x, y1, 1);
				spline.evaluate(x, y2, 2);

				for(size_t i = 0; i < pts.size(); ++i)
					output << pts[i].x() << "," << pts[i].y() << "," << y[i] << "," << y1[i] << "," << y2[i] << "\n";

			} catch(const std::exception& ex) {
				std::cerr << ex.what() << "\n";
			}
		}
	}
}



