/*
 * geog590.cpp
 *
 *  Created on: Apr 1, 2019
 *      Author: rob
 */

#include <vector>
#include <iostream>

#include "math/SmoothSpline.hpp"
#include "geog599/TrajectoryPlanner.hpp"

using namespace uav::geog599;

double stddev(std::vector<Pt>& pts) {
	double s = 0;
	for(Pt& p : pts)
		s += p.y();
	s /= pts.size();
	double v = 0;
	for(Pt& p : pts)
		v += std::pow(p.y() - s, 2.0);
	return std::sqrt(v);
}

int main(int argc, char** argv) {

	std::vector<Pt> pts1 = {Pt(1,0, 0), Pt(2, 0, 0), Pt(3, 3, 0), Pt(4, 6, 0), Pt(5, 9, 0), Pt(6, 12, 0), Pt(7, 12, 0), Pt(8, 12, 0), Pt(9, 12, 0), Pt(10, 12, 0)};
	std::vector<Pt> pts2 = {Pt(11, 7, 0), Pt(12, 7, 0), Pt(13, 7, 0), Pt(14, 6, 0), Pt(15, 5, 0), Pt(16, 4, 0), Pt(17, 3, 0), Pt(18, 2, 0), Pt(19, 1, 0), Pt(20, 0, 0)};

	double st1 = stddev(pts1);
	double st2 = stddev(pts2);

	std::vector<double> be(2);

	uav::math::SmoothSpline<Pt> sp;

	sp.fit(pts1, 1, 2);
	if(!sp.valid())
		std::cout << sp.errorStr() << "\n";

	std::vector<double> x;
	std::vector<double> y;

	sp.linspace(pts1[0].x(), pts1[pts1.size() - 1].x(), x, 10);
	y.resize(x.size());

	sp.evaluate(x, y, 0);

	std::cout << "Altitude\n";
	for(size_t i = 0; i < x.size(); ++i)
		std::cout << "(" << x[i] << "," << y[i] << ") ";
	std::cout << "\n";

	be[0] = y[y.size() - 1];

	sp.evaluate(x, y, 1);

	std::cout << "Velocity\n";
	for(size_t i = 0; i < x.size(); ++i)
		std::cout << "(" << x[i] << "," << y[i] << ") ";
	std::cout << "\n";

	be[1] = y[y.size() - 1];

	sp.evaluate(x, y, 2);

	std::cout << "Acceleration\n";
	for(size_t i = 0; i < x.size(); ++i)
		std::cout << "(" << x[i] << "," << y[i] << ") ";
	std::cout << "\n";

	//be[2] = y[y.size() - 1];



	sp.fit(pts2, 1, 15, be);
	if(!sp.valid())
		std::cout << sp.errorStr() << "\n";

	sp.linspace(pts2[0].x(), pts2[pts2.size() - 1].x(), x, 10);
	y.resize(x.size());

	sp.evaluate(x, y, 0);

	std::cout << "Altitude\n";
	for(size_t i = 0; i < x.size(); ++i)
		std::cout << "(" << x[i] << "," << y[i] << ") ";
	std::cout << "\n";

	sp.evaluate(x, y, 1);

	std::cout << "Velocity\n";
	for(size_t i = 0; i < x.size(); ++i)
		std::cout << "(" << x[i] << "," << y[i] << ") ";
	std::cout << "\n";

	sp.evaluate(x, y, 2);

	std::cout << "Acceleration\n";
	for(size_t i = 0; i < x.size(); ++i)
		std::cout << "(" << x[i] << "," << y[i] << ") ";
	std::cout << "\n";


}

