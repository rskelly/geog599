/*
 * pipeline.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#include <unistd.h>

#include <string>

#include "TrajectoryPlanner.hpp"
#include "HullPointFilter.hpp"
#include "GeomPointFilter.hpp"
#include "PointSorter.hpp"
#include "sim/TxtPointSource.hpp"

using namespace uav;
using namespace uav::sim;

int main(int argc, char** argv) {

	std::string ptsFile;

	PointSorter<Pt> psort;
	TxtPointSource<Pt> tps(ptsFile);
	HullPointFilter<Pt> hpf(10.0);
	GeomPointFilter<Pt> gpf;

	gpf.setNextFilter(&hpf);

	TrajectoryPlanner<Pt> tp;
	tp.setPointFilter(&gpf);
	tp.setPointSorter(&psort);
	tp.setPointSource(&tps);

	double y = 0;
	double speed = 10.0; // m/s
	int delay = 100;

	while(true) {

		gpf.setMinY(y);
		tp.generate();

		y += speed * ((double) delay / 1000);
		usleep(delay * 1000);
	}
}
