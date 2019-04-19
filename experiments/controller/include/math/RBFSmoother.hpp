/*
 * RBF.hpp
 *
 *  Created on: Apr 5, 2019
 *      Author: rob
 */

#ifndef INCLUDE_MATH_RBFSMOOTHER_HPP_
#define INCLUDE_MATH_RBFSMOOTHER_HPP_

#include <vector>
#include <cmath>

#include "math/Util.hpp"

namespace uav {
namespace math {

template <class P>
class RBF {
private:
	std::vector<P> m_output;
	double m_spacing;
	double m_xmin;
	double m_xmax;

	double m_2pi;

public:

	RBF() :
		m_spacing(1),
		m_xmin(0), m_xmax(0),
		m_2pi(std::sqrt(2.0 * M_PI)) {
	}


	double fn(double t, double u, double sigma) {
		return (1.0 / (sigma * m_2pi)) * std::exp(std::pow((t - u) / sigma, 2.0) * -0.5)	;
	}

	template <class Iter>
		bool fit(Iter begin, Iter end, double s,
				const std::vector<double>& bc = {}, const std::vector<double>& ec = {}) {
		std::vector<P> pts(begin, end);
		fit(pts, s, m_spacing);
	}

	double min() const {
		return m_xmin;
	}

	double max() const {
		return m_xmax;
	}

	bool fit(const std::vector<P>& pts, double sigma, double spacing) {

		m_xmin = pts[0].y();
		m_xmax = pts[pts.size() - 1].y();
		std::vector<double> x = Util::linspace(m_xmin, m_xmax, (int) std::ceil((m_xmax - m_xmin) / spacing));
		m_output.resize(x.size());
		for(P& p : m_output)
			p.z(0);

		for(size_t i = 0; i < x.size(); ++i) {
			m_output[i].y(x[i]);
			for(size_t j = 0; j < pts.size(); ++j) {
				const P& pt = pts[j];
				double f = fn(x[i], pt.y(), sigma);
				m_output[i].z(m_output[i].z() + f * pt.z());
			}
		}

		return true;
	}

	std::vector<double> knots() const {
		std::vector<double> k;
		for(const P& p : m_output)
			k.push_back(p.y());
		return k;
	}

	const std::vector<P>& pknots() const {
		return m_output;
	}

	bool valid() {
		return !m_output.empty();
	}

	/**
	 * Evaluate the spline at the given positions in x for the given derivative (default 0).
	 *
	 * @param x The x-coordinates.
	 * @param y The y-coordinates (output).
	 * @param derivative The derivative to evaluate. Defaults to zero, the original function.
	 */
	bool evaluate(const std::vector<double>& x, std::vector<double>& y, int derivative = 0) {
		if(!valid()) {
			std::fill(y.begin(), y.end(), 0);
			return false;
		}
		if(derivative == 0) {
			size_t j = 0;
			for(size_t i = 0; i < x.size(); ++i) {
				double x0 = x[i];
				while(x0 < m_output[j].y() && j > 0)
					--j;
				while(j < m_output.size() - 1 && x0 >= m_output[j + 1].y())
					++j;
				if(x0 == m_output[j].y()) {
					y[i] = m_output[j].z();
				} else if(j == m_output.size() - 1) {
					y[i] = m_output[j - 1].z() + (x0 - m_output[j - 1].y()) / (m_output[j].y() - m_output[j - 1].y()) * (m_output[j].z() - m_output[j - 1].z());
				} else {
					y[i] = m_output[j].z() + (x0 - m_output[j].y()) / (m_output[j + 1].y() - m_output[j].y()) * (m_output[j + 1].z() - m_output[j].z());
				}
			}
		} else if(derivative == 1){
			size_t j = 0;
			for(size_t i = 0; i < x.size(); ++i) {
				double x0 = x[i];
				while(x0 < m_output[j].y() && j > 0)
					--j;
				while(j < m_output.size() - 1 && x0 >= m_output[j + 1].y())
					++j;
				if(x0 == m_output[j].y()) {
					y[i] = 0;
				} else if(j == m_output.size() - 1) {
					y[i] = (m_output[j].z() - m_output[j - 1].z()) / (m_output[j].y() - m_output[j - 1].y());
				} else {
					y[i] = (m_output[j + 1].z() - m_output[j].z()) / (m_output[j + 1].y() - m_output[j].y());
				}
			}
		} else {
			return false;
		}
		return true;
	}

	/**
	 * Evaluate the spline at the given position in x for the given derivative (default 0).
	 * @param x The x-coordinate.
	 * @param y The y-coordinate (output).
	 * @param derivative The derivative to evaluate. Defaults to zero, the original function.
	 */
	bool evaluate(double x, double& y, int derivative = 0) {
		if(!valid()) {
			y = 0;
			return false;
		}
		std::vector<double> xx(1), yy(1);
		xx[0] = x;
		if(evaluate(xx, yy, derivative)) {
			y = yy[0];
			return true;
		}
		return false;
	}

	/**
	 * Return a list of the derivative values at the given location,
	 * for the derivative orders in k (starting with zero).
	 *
	 * @param at The x-coordinate.
	 * @param k A vector containing derivative indices.
	 * @return A vector of derivative values.
	 */
	std::vector<double> derivatives(double at, std::vector<int> k) {
		if(!valid())
			return {};
		static std::vector<double> xx(1);
		static std::vector<double> yy(1);
		std::vector<double> result(k.size());
		xx[0] = at;
		for(size_t i = 0; i < k.size(); ++i) {
			evaluate(xx, yy, k[i]);
			result[i] = yy[0];
		}
		return result;
	}

};
} // math
} // uav


#endif /* INCLUDE_MATH_RBFSMOOTHER_HPP_ */
