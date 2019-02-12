/*
 * SmoothSpline.hpp
 *
 *  Created on: Feb 11, 2019
 *      Author: rob
 */

#ifndef INCLUDE_MATH_SMOOTHSPLINE_HPP_
#define INCLUDE_MATH_SMOOTHSPLINE_HPP_

#include <vector>
#include <cmath>


/*
c   d is a parameter containing the observation weights.
c   this may either be an array of length n or a scalar
c   (interpreted as a constant). the value of d
c   corresponding to the observation (x(k),y(k)) should
c   be an approximation to the standard deviation of error.
c
c   isw contains a switch indicating whether the parameter
c   d is to be considered a vector or a scalar,
c          = 0 if d is an array of length n,
c          = 1 if d is a scalar.
c
c   s contains the value controlling the smoothing. this
c   must be non-negative. for s equal to zero, the
c   subroutine does interpolation, larger values lead to
c   smoother funtions. if parameter d contains standard
c   deviation estimates, a reasonable value for s is
c   float[n - 1].
c
c   eps contains a tolerance on the relative precision to
c   which s is to be interpreted. this must be greater than
c   or equal to zero and less than or equal to one. a
c   reasonable value for eps is sqrt(2./float[n - 1]).
c
c   ys is an array of length at least n.
c
c   ysp is an array of length at least n.
c
c   sigma contains the tension factor. this value indicates
c   the degree to which the first derivative part of the
c   smoothing functional is emphasized. if sigma is nearly
c   zero (e. g. .001) the resulting curve is approximately a
c   cubic spline. if sigma is large (e. g. 50.) the
c   resulting curve is nearly a polygonal line. if sigma
c   equals zero a cubic spline results. a standard value for
c   sigma is approximately 1.
c
c and
c
c   temp is an array of length at least 9*n which is used
c   for scratch storage.
c
c on output--
c
c   ys contains the smoothed ordinate values.
c
c   ysp contains the values of the second derivative of the
c   smoothed curve at the given nodes.
c
 */

extern "C" {
	void curvs_(const int* n, const float* x, const float* y, const float* d,
			const int* isw, const float* s, const float* eps,
			float* ys, float* ysp,
			const float* sigma, float* temp, int* ierr);
}


namespace uav {
namespace math {

class SplinePiece {
public:
	double x0;
	double y0;
	double x1;
	double y1;
	double d0;
	double d1;
	SplinePiece(double x0, double y0, double x1, double y1, double d0, double d1)
		: x0(x0), y0(y0), x1(x1), y1(y1), d0(d0), d1(d1) {}
};

template <class P>
class SmoothSpline {
public:

	/**
	 * @param pts 		A list of points with an x and y property. X is the abscissa; y is the ordinate.
	 * @param weight 	A scalar giving the weight for each data point.
	 * @param s 		The smoothing factor.
	 * @param sigma 	The tension parameter.
	 * @param out 		The output of the function; a list of "curve" objects containing the pair
	 * 					of knots and the second derivative at each.
	 */
	void fit(const std::vector<P>& pts, double weight, double s, double sigma,
			std::vector<SplinePiece>& out) {

		std::vector<double> weights(pts.size());
		for(size_t i = 0; i < weights.size(); ++i)
			weights[i] = weight;

		fit(pts, weights, s, sigma, out);
	}

	/**
	 * @param pts 		A list of points with an x and y property. X is the abscissa; y is the ordinate.
	 * @param weights 	A list of weights at each data point.
	 * @param s 		The smoothing factor.
	 * @param sigma 	The tension parameter.
	 * @param out 		The output of the function; a list of "curve" objects containing the pair
	 * 					of knots and the second derivative at each.
	 */
	void fit(const std::vector<P>& pts, const std::vector<double>& weights, double s, double sigma,
			std::vector<SplinePiece>& out) {

		if(pts.size() < 2 || pts.size() != weights.size())
			throw std::runtime_error("Weights and points must have the same length and be more than 2.");

		// Inputs
		int isw = 0;								// Weights are an array (1 would be scalar)
		int n = (int) pts.size();					// Number of points.
		std::vector<double> x(n); 					// Abscissae
		std::vector<double> y(n); 					// Ordinates
		double eps = std::sqrt(2.0f / (double) n);	// Epsilon

		// Temp
		std::vector<double> temp0(n);	// Temp storage.
		std::vector<double> temp1(n);	// Temp storage.
		std::vector<double> temp2(n);	// Temp storage.
		std::vector<double> temp3(n);	// Temp storage.
		std::vector<double> temp4(n);	// Temp storage.
		std::vector<double> temp5(n);	// Temp storage.
		std::vector<double> temp6(n);	// Temp storage.
		std::vector<double> temp7(n);	// Temp storage.
		std::vector<double> temp8(n);	// Temp storage.

		// Outputs
		int ierr = 0;						// Error return.
		std::vector<double> ys(n);			// Smoothed ordinates.
		std::vector<double> ysp(n);			// Smoothed derivatives.

		for(int i = 0; i < n; ++i) {
			x[i] = pts[i].x();
			y[i] = pts[i].y();
		}

		curvss(n, x, y, weights, isw, s, eps, ys, ysp, sigma, ierr);

		switch(ierr) {
		case 1:
			throw std::runtime_error("n cannot be less than 2.");
		case 2:
			throw std::runtime_error("s must be positive.");
		case 3:
			throw std::runtime_error("eps must be positive and <= 1.");
		case 4:
			throw std::runtime_error("x values are not strictly increasing.");
		case 5:
			throw std::runtime_error("d value must be positive.");
		}

		for(int i = 1; i < n; ++i)
			out.emplace_back(x[i - 1], ys[i - 1], x[i], ys[i], ysp[i - 1], ysp[i]);

	}


/*
c
c                                 coded by alan kaylor cline
c                           from fitpack -- january 26, 1987
c                        a curve and surface fitting package
c                      a product of pleasant valley software
c                  8603 altus cove, austin, texas 78759, usa
c
c this subroutine determines the parameters necessary to
c compute a smoothing spline under tension. for a given
c increasing sequence of abscissae (x[i]), i = 1,..., n and
c associated ordinates (y[i]), i = 1,..., n, the function
c determined minimizes the summation from i = 1 to n-1 of
c the square of the second derivative of f plus sigma
c squared times the difference of the first derivative of f
c and (f(x[i + 1])-f(x[i]))/(x[i + 1]-x[i]) squared, over all
c functions f with two continuous derivatives such that the
c summation of the square of (f(x[i])-y[i])/d[i] is less
c than or equal to a given constant s, where (d[i]), i = 1,
c ..., n are a given set of observation weights. the
c function determined is a spline under tension with third
c derivative discontinuities at (x[i]), i = 2,..., n-1. for
c actual computation of points on the curve it is necessary
c to call the function curv2.
c
c on input--
c
c   n is the number of values to be smoothed (n.ge.2).
c
c   x is an array of the n increasing abscissae of the
c   values to be smoothed.
c
c   y is an array of the n ordinates of the values to be
c   smoothed, (i. e. y(k) is the functional value
c   corresponding to x(k) ).
c
c   d is a parameter containing the observation weights.
c   this may either be an array of length n or a scalar
c   (interpreted as a constant). the value of d
c   corresponding to the observation (x(k),y(k)) should
c   be an approximation to the standard deviation of error.
c
c   isw contains a switch indicating whether the parameter
c   d is to be considered a vector or a scalar,
c          = 0 if d is an array of length n,
c          = 1 if d is a scalar.
c
c   s contains the value controlling the smoothing. this
c   must be non-negative. for s equal to zero, the
c   subroutine does interpolation, larger values lead to
c   smoother funtions. if parameter d contains standard
c   deviation estimates, a reasonable value for s is
c   float[n - 1].
c
c   eps contains a tolerance on the relative precision to
c   which s is to be interpreted. this must be greater than
c   or equal to zero and less than equal or equal to one. a
c   reasonable value for eps is sqrt(2./float[n - 1]).
c
c   ys is an array of length at least n.
c
c   ysp is an array of length at least n.
c
c   sigma contains the tension factor. this value indicates
c   the degree to which the first derivative part of the
c   smoothing functional is emphasized. if sigma is nearly
c   zero (e. g. .001) the resulting curve is approximately a
c   cubic spline. if sigma is large (e. g. 50.) the
c   resulting curve is nearly a polygonal line. if sigma
c   equals zero a cubic spline results. a standard value for
c   sigma is approximately 1.
c
c and
c
c   td, tsd1, hd, hsd1, hsd2, rd, rsd1, rsd2, and v are
c   arrays of length at least n which are used for scratch
c   storage.
c
c on output--
c
c   ys contains the smoothed ordinate values.
c
c   ysp contains the values of the second derivative of the
c   smoothed curve at the given nodes.
c
c   ierr contains an error flag,
c        = 0 for normal return,
c        = 1 if n is less than 2,
c        = 2 if s is negative,
c        = 3 if eps is negative or greater than one,
c        = 4 if x-values are not strictly increasing,
c        = 5 if a d-value is non-positive.
c
c and
c
c   n, x, y, d, isw, s, eps, and sigma are unaltered.
c
c this subroutine references package modules terms and
c snhcsh.
c
c-----------------------------------------------------------
c

 */
    void curvss (int n,
    		const std::vector<double>& x,const std::vector<double>& y, const std::vector<double>& d,
			int isw, double s,double eps, std::vector<double>& ys, std::vector<double>& ysp, double sigma, int& ierr) {

		std::vector<double> td(n);
		std::vector<double> tsd1(n);
		std::vector<double> hd(n);
		std::vector<double> hsd1(n);
		std::vector<double> hsd2(n);
		std::vector<double> rd(n);
		std::vector<double> rsd1(n);
		std::vector<double> rsd2(n);
		std::vector<double> v(n);

		double p, rdim1, yspim2, sigmap;
		int nm1, nm3;
		double delxi1, delyi1, dim1, di;
		double delxi, delyi;
		double sl, su;
		double betapp, betap, alphap, beta, alpha;
		double hsd1p, hdim1, hdi;
		double rsd2i, rsd1i;
		double sum;
		double f, g, h, i, wim2, wim1, tui, wi, step;

		if (n < 2)
			goto _16;

		if (s < 0.)
			goto _17;

		if (eps < 0. || eps > 1.)
			goto _18;

		ierr = 0;
		p = 0.;
		v[0] = 0.;
		v[n - 1] = 0.;
		ysp[0] = 0.;
		ysp[n - 1] = 0.;

		if (n == 2)
			goto _14;

		rsd1[0] = 0.;
		rd[0] = 0.;
		rsd2[n - 1] = 0.;
		rdim1 = 0.;
		yspim2 = 0.;

	// c denormalize tension factor

		sigmap = std::abs(sigma) * (double)(n-1) / (x[n - 1] - x[0]);

	// c form t matrix and second differences of y into ys

		nm1 = n - 1;
		nm3 = n - 3;
		delxi1 = 1.;
		delyi1 = 0.;
		dim1 = 0.;
		for(int i = 0; i < nm1; ++i) {
		  delxi = x[i + 1] - x[i];

		  if (delxi <= 0.)
			  goto _19;

		  delyi = (y[i + 1] - y[i]) / delxi;
		  ys[i] = delyi - delyi1;

		  terms (di, tsd1[i + 1], sigmap, delxi);

		  td[i] = di + dim1;
		  hd[i] = -(1. / delxi + 1. / delxi1);
		  hsd1[i + 1] = 1. / delxi;
		  delxi1 = delxi;
		  delyi1 = delyi;
		  dim1 = di;
		}

	// c calculate lower and upper tolerances

		sl = s * (1. - eps);
		su = s * (1. + eps);

		if (isw == 1)
			goto _3;

	// c form h matrix - d array

		if (d[0] <= 0. || d[1] <= 0.)
			goto _20;

		betapp = 0.;
		betap = 0.;
		alphap = 0.;
		for(int i = 1; i < nm1; ++i) {
		  alpha = hd[i] * d[i] * d[i];

		  if (d[i + 1] <= 0.)
			  goto _20;

		  beta = hsd1[i + 1] * d[i + 1] * d[i + 1];
		  hd[i] = std::pow(hsd1[i] * d[i-1], 2) + alpha * hd[i] + beta * hsd1[i + 1];
		  hsd2[i] = hsd1[i] * betapp;
		  hsd1[i] = hsd1[i] * (alpha + alphap);
		  alphap = alpha;
		  betapp = betap;
		  betap = beta;
		}

		goto _5;

	// c form h matrix - d constant

_3:
		if (d[0] <= 0.)
			goto _20;

		sl = d[0] * d[0] * sl;
		su = d[0] * d[0] * su;
		hsd1p = 0.;
		hdim1 = 0.;
		for(int i = 1; i < nm1; ++i) {
		  hdi = hd[i];
		  hd[i] = hsd1[i] * hsd1[i] + hdi * hdi + hsd1[i + 1] * hsd1[i + 1];
		  hsd2[i] = hsd1[i] * hsd1p;
		  hsd1p = hsd1[i];
		  hsd1[i] = hsd1p * (hdi + hdim1);
		  hdim1 = hdi;
		}

	// c top of iteration
	// c cholesky factorization of p*t+h into r

_5:
		for(int i = 1; i < nm1; ++i) {
		  rsd2i = hsd2[i];
		  rsd1i = p * tsd1[i] + hsd1[i] - rsd2i * rsd1[i - 1];
		  rsd2[i] = rsd2i * rdim1;
		  rdim1 = rd[i - 1];
		  rsd1[i] = rsd1i * rdim1;
		  rd[i] = 1. / (p * td[i] + hd[i] - rsd1i * rsd1[i] - rsd2i * rsd2[i]);
		  ysp[i] = ys[i] - rsd1[i] * ysp[i - 1] - rsd2[i] * yspim2;
		  yspim2 = ysp[i - 1];
		}

	// c back solve of r(transpose)* r * ysp = ys

		ysp[nm1 - 1] = rd[nm1 - 1] * ysp[nm1 - 1];

		if (n == 3)
			goto _8;

		for(int ibak = 0; ibak < nm3; ++ibak) {
		  i = (nm1 - 1) - ibak;
		  ysp[i] = rd[i] * ysp[i] - rsd1[i + 1] * ysp[i + 1] - rsd2[i + 2] * ysp[i + 2];
		}

_8:
		sum = 0.;
		delyi1 = 0.;

		if (isw == 1)
			goto _10;

	// c calculation of residual norm
	// c  - d array

		for(int i = 0; i < nm1; ++i) {
		  delyi = (ysp[i + 1] - ysp[i]) / (x[i + 1] - x[i]);
		  v[i] = (delyi - delyi1) * d[i] * d[i];
		  sum = sum + v[i] * (delyi - delyi1);
		  delyi1 = delyi;
		}

		v[n - 1] = -delyi1 * d[n - 1] * d[n - 1];
		goto _12;

	// c calculation of residual norm
	// c  - d constant

_10:
		for(int i = 0; i < nm1; ++i) {
		  delyi = (ysp[i + 1] - ysp[i]) / (x[i + 1] - x[i]);
		  v[i] = delyi - delyi1;
		  sum = sum + v[i] * (delyi - delyi1);
		  delyi1 = delyi;
		}
		v[n - 1] = -delyi1;

_12:
		sum = sum - v[n - 1] * delyi1;

	// c test for convergence

		if (sum <= su)
			goto _14;

	// c calculation of newton correction

		f = 0.;
		g = 0.;
		wim2 = 0.;
		wim1 = 0.;

		for(int i = 1; i < nm1; ++i) {
		  tui = tsd1[i] * ysp[i - 1] + td[i] * ysp[i] + tsd1[i + 1] * ysp[i + 1];
		  wi = tui - rsd1[i] * wim1 - rsd2[i] * wim2;
		  f = f + tui * ysp[i];
		  g = g + wi * wi * rd[i];
		  wim2 = wim1;
		  wim1 = wi;
		}
		h = f - p * g;

		if (h <= 0.)
			goto _14;

	// c update p - newton step

		step = (sum - std::sqrt(sum * sl)) / h;

		if (sl != 0.)
			step = step * std::sqrt(sum / sl);
		p = p + step;
		goto _5;

	// c store smoothed y-values and second derivatives

_14:
		for(int i = 0; i < n; ++i) {
		  ys[i] = y[i] - v[i];
		  ysp[i] = p * ysp[i];
		}
		return;

	// c n less than 2

_16:
		ierr = 1;
		return;

	// c s negative

_17:
		ierr = 2;
		return;

	// c eps negative or greater than 1

_18:
		ierr = 3;
		return;

	// c x-values not strictly increasing

_19:
		ierr = 4;
		return;

	// c weight non-positive

_20:
		ierr = 5;
		return;
	}

	/*
	c
		real diag,sdiag,sigma,del
	c
	c                                 coded by alan kaylor cline
	c                           from fitpack -- january 26, 1987
	c                        a curve and surface fitting package
	c                      a product of pleasant valley software
	c                  8603 altus cove, austin, texas 78759, usa
	c
	c this subroutine computes the diagonal and superdiagonal
	c terms of the tridiagonal linear system associated with
	c spline under tension interpolation.
	c
	c on input--
	c
	c   sigma contains the tension factor.
	c
	c and
	c
	c   del contains the step size.
	c
	c on output--
	c
	c                sigma*del*cosh(sigma*del) - sinh(sigma*del)
	c   diag = del*--------------------------------------------.
	c                     (sigma*del)**2 * sinh(sigma*del)
	c
	c                   sinh(sigma*del) - sigma*del
	c   sdiag = del*----------------------------------.
	c                (sigma*del)**2 * sinh(sigma*del)
	c
	c and
	c
	c   sigma and del are unaltered.
	c
	c this subroutine references package module snhcsh.
	c
	c-----------------------------------------------------------
	c
	 */
    void terms (double& diag, double& sdiag, double sigma, double del) {
    	if (sigma == 0.) {
    		double sigdel = sigma * del;
    		diag = del * (sigma * del * std::cosh(sigma * del) - std::sinh(sigma * del)) / (std::pow(sigma * del, 2.0) * std::sinh(sigma * del));
    		sdiag = del * (std::sinh(sigma * del) - sigma * del) / (std::pow(sigma * del, 2.0) * std::sinh(sigma * del));
		} else {
			diag = del / 3.;
			sdiag = del / 6.;
		}
    }

};

} // math
} // uav



#endif /* INCLUDE_MATH_SMOOTHSPLINE_HPP_ */
