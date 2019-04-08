/*
 * Smoother.hpp
 *
 *  Created on: Apr 8, 2019
 *      Author: rob
 */

#ifndef INCLUDE_MATH_SMOOTHER_HPP_
#define INCLUDE_MATH_SMOOTHER_HPP_

#include <vector>

namespace uav {
namespace math {

template <class P>
class Smoother {
protected:
	int m_err;				///<! The error code.
	std::string m_errMsg;	///<! The error message.

	/**
	 * Set the error code and message.
	 *
	 * @param err The error code.
	 * @param msg The error message.
	 */
	void setError(int err, const std::string& msg) {
		m_err = err;
		m_errMsg = msg;
	}

public:

	/**
	 * Returns true if the last operation completed properly.
	 *
	 * @return True if the last operation completed properly.
	 */
	virtual bool valid() const = 0;

	/**
	 * Return the minimum abscissa value.
	 *
	 * @return The minimum abscissa value.
	 */
	virtual double min() const = 0;

	/**
	 * Return the maximum abscissa value.
	 *
	 * @return The maximum abscissa value.
	 */
	virtual double max() const = 0;

	/**
	 * Return the error code.
	 *
	 * @return  The error code.
	 */
	int error() const {
		return m_err;
	}

	/**
	 * Return the error message.
	 *
	 * @return The error message.
	 */
	const std::string& errorMessage() const {
		return m_errMsg;
	}

	/**
	 * Fit the list of points.
	 *
	 * @param begin A start iterator.
	 * @param end An end iterator.
	 * @return True, if successful.
	 */
	template <class Iter>
	bool fit(Iter begin, Iter end) {
		std::vector<P> pts(begin, end);
		fit(pts);
	}

	/**
	 * Fit the list of points.
	 *
	 * @param pts A list of points.
	 * @return True, if successful.
	 */
	virtual bool fit(const std::vector<P>& pts) = 0;

	/**
	 * Return a list of the abscissae of the knots.
	 *
	 * @return A list of the abscissae of the knots.
	 */
	virtual const std::vector<double>& knots() const = 0;

	/**
	 * Return a list of the coefficients.
	 *
	 * @return A list of the coefficients.
	 */
	virtual const std::vector<double>& coefficients() const = 0;

	/**
	 * Evaluate the function at the given positions in x for the given derivative (default 0).
	 *
	 * @param x The x-coordinates.
	 * @param y The y-coordinates (output).
	 * @param derivative The derivative to evaluate. Defaults to zero, the original function.
	 */
	virtual bool evaluate(const std::vector<double>& x, std::vector<double>& y, int derivative = 0) = 0;

	/**
	 * Evaluate the function at the given position in x for the given derivative (default 0).
	 * @param x The x-coordinate.
	 * @param y The y-coordinate (output).
	 * @param derivative The derivative to evaluate. Defaults to zero, the original function.
	 */
	virtual bool evaluate(double x, double& y, int derivative = 0) = 0;

	/**
	 * Return a list of the derivative values at the given location,
	 * for the derivative orders in k (starting with zero).
	 *
	 * @param at The x-coordinate.
	 * @param k A vector containing derivative indices.
	 * @return A vector of derivative values.
	 */
	virtual std::vector<double> derivatives(double at, std::vector<int> k) = 0;

	virtual ~Smoother() {}
};

} // math
} // uav



#endif /* INCLUDE_MATH_SMOOTHER_HPP_ */
