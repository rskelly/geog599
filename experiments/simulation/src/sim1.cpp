

#include <iostream>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <iomanip>

#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"

double time() {
	timeval time;
	gettimeofday(&time, NULL);
	return (double) time.tv_sec + ((double) time.tv_usec / 1000000);
}

int main(int argc, char** argv) {

	SimPlatform p;

	std::cerr << std::setprecision(12);

	double start = time();

	while(true) {
		double current = time() - start;

		p.update(current);

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
}
