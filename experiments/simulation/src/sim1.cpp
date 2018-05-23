

#include <iostream>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <iomanip>

#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"

#include <Eigen/Core>
#include "util.hpp"

double time() {
	timeval time;
	gettimeofday(&time, NULL);
	return (double) time.tv_sec + ((double) time.tv_usec / 1000000);
}


void testMath() {
	double a = MatrixUtil::toRad(45);
	Eigen::Vector3d vec(1, 0, 0);
	std::cerr << vec[0] << ", " << vec[1] << ", " << vec[2] << "\n";
	Eigen::Matrix3d mtx = MatrixUtil::rotFromAxisAngle(vec, a);
	std::cerr << mtx(0, 0) << ", " << mtx(0, 1) << ", " << mtx(0, 2) << "\n";
	std::cerr << mtx(1, 0) << ", " << mtx(1, 1) << ", " << mtx(1, 2) << "\n";
	std::cerr << mtx(2, 0) << ", " << mtx(2, 1) << ", " << mtx(2, 2) << "\n";
	Eigen::Vector3d input(2, 1, 3);
	std::cerr << input[0] << ", " << input[1] << ", " << input[2] << "\n";
	Eigen::Vector3d out = mtx * input;
	std::cerr << out[0] << ", " << out[1] << ", " << out[2] << "\n";
}

int main(int argc, char** argv) {

	//testMath();

	SimPlatform p;

	std::cerr << std::setprecision(12);

	double start = time();

	while(true) {
		double current = time() - start;

		p.update(current);

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

}
