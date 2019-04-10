/*
 * RBF.hpp
 *
 *  Created on: Apr 5, 2019
 *      Author: rob
 */

#ifndef INCLUDE_MATH_IDWSMOOTHER_HPP_
#define INCLUDE_MATH_IDWSMOOTHER_HPP_

#include <vector>
#include <cmath>

#include "math/Util.hpp"
#include "math/Smoother.hpp"

namespace uav {
namespace math {

template <class P>
class IDWSmoother : public Smoother<P> {
private:
	std::vector<P> m_output;	///<! A list of the output values, with abscissa spacing given by spacing.
	double m_radius;			///<! The radius of the smoothing kernel.
	double m_xmin;				///<! The minimum abscissa value.
	double m_xmax;				///<! The maximum abscissa value.
	double m_exponent;			///<! The weighting exponent.
	double m_spacing;			///<! The spacing between output abscissae.

	std::vector<double> m_knots;
	std::vector<double> m_coefs;
public:

	/***
	 * Build a smoother with the given properties.
	 *
	 * @param radius The radius of the kernel.
	 * @param exponent The exponent of the weight function.
	 * @param spacing The spacing between output abscissae.
	 */
	IDWSmoother(double radius = 20, double exponent = 2, double spacing = 1) :
		m_radius(radius),
		m_xmin(0), m_xmax(0),
		m_exponent(exponent),
		m_spacing(spacing) {
	}

	void setSpacing(double spacing) {
		m_spacing = spacing;
	}

	double spacing() const {
		return m_spacing;
	}

	void setRadius(double radius) {
		m_radius = radius;
	}

	double radius() const {
		return m_radius;
	}

	double min() const {
		return m_xmin;
	}

	double max() const {
		return m_xmax;
	}

	bool valid() const {
		return this->m_err == 0 && !m_output.empty();
	}

	bool fit(const std::vector<P>& pts) {

		if(pts.empty())
			return false;

		m_xmin = pts[0].y();
		m_xmax = pts[pts.size() - 1].y();

		m_knots = Util::linspace(m_xmin, m_xmax, m_spacing);
		m_output.resize(m_knots.size());

		for(size_t i = 0; i < m_knots.size(); ++i) {
			m_output[i].y(m_knots[i]);
			double s = 0, w = 0;
			for(size_t j = 0; j < pts.size(); ++j) {
				const P& pt = pts[j];
				double d = std::pow(std::abs(pt.y() - m_knots[i]), m_exponent);
				if(d > m_radius * m_radius)
					continue;
				if(d == 0) {
					s = pt.z();
					w = 1;
					break;
				} else {
					double w0 = 1.0 / d;
					w += w0;
					s += pt.z() * w0;
				}
			}
			m_output[i].z(s / w);
		}
		this->m_err = 0;
		return true;
	}

	const std::vector<double>& knots() const {
		return m_knots;
	}

	const std::vector<double>& coefficients() const {
		return m_coefs;
	}

	const std::vector<P>& output() const {
		return m_output;
	}

	bool evaluate(const std::vector<double>& x, std::vector<double>& y, int derivative = 0) {
		if(!valid()) {
			std::fill(y.begin(), y.end(), 0);
			return false;
		}
		double minx = m_output[0].y();
		double maxx = m_output[m_output.size() - 1].y();
		if(derivative == 0) {
			size_t j = 0;
			for(size_t i = 0; i < x.size(); ++i) {
				double x0 = x[i];
				if(x0 < minx || x0 > maxx) {
					y[i] = std::nan("");
					continue;
				}
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
				if(x0 < minx || x0 > maxx) {
					y[i] = std::nan("");
					continue;
				}
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


#endif /* INCLUDE_MATH_IDWSMOOTHER_HPP_ */
