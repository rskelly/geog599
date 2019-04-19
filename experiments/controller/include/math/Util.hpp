/*
 * Util.hpp
 *
 *  Created on: Apr 5, 2019
 *      Author: rob
 */

#ifndef INCLUDE_MATH_UTIL_HPP_
#define INCLUDE_MATH_UTIL_HPP_

#include <vector>

namespace uav {
namespace math {

class Util {
public:

	/**
	 * Populate the vector with regularly spaced doubles, according to count.
	 *
	 * @param x0 The starting x.
	 * @param x1 The ending x.
	 * @param lst The output list of values.
	 * @param count The number of items.
	 */
	static std::vector<double> linspace(double x0, double x1, int count) {
		if(count < 2) count = 2;
		std::vector<double> lst(count);
		if(count == 2) {
			lst[0] = x0;
			lst[1] = x1;
		} else {
			lst.resize(count);
			double dist = (x1 - x0) / (count - 1);
			for(size_t i = 0; i < count - 1; ++i)
				lst[i] = x0 + dist * i;
			lst[count - 1] = x1;
		}
		return lst;
	}

	static std::vector<double> linspace(double x0, double x1, double spacing) {
		int count = (int) ((x1 - x0) / spacing);
		if(count < 2) count = 2;
		if(count == 2) {
			return {x0, x1};
		} else {
			std::vector<double> result(count);
			for(int i = 0; i < count - 1; ++i)
				result[i] = x0 + spacing * i;
			result[count - 1] = x1;
			return result;
		}
	}

};

} // math
} // uav



#endif /* INCLUDE_MATH_UTIL_HPP_ */
