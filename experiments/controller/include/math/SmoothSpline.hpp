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

#define NOT_RUN 1000 // Just an error value to show that a spline hasn't been computed yet.

extern "C" {

	/*
	 * c  parameters:
		c   iopt  : integer flag. on entry iopt must specify whether a weighted
		c           least-squares spline curve (iopt=-1) or a smoothing spline
		c           curve (iopt=0 or 1) must be determined.if iopt=0 the routine
		c           will start with an initial set of knots t(i)=ub,t(i+k+1)=ue,
		c           i=1,2,...,k+1. if iopt=1 the routine will continue with the
		c           knots found at the last call of the routine.
		c           attention: a call with iopt=1 must always be immediately
		c           preceded by another call with iopt=1 or iopt=0.
		c           unchanged on exit.
		c   idim  : integer. on entry idim must specify the dimension of the
		c           curve. 0 < idim < 11.
		c           unchanged on exit.
		c   m     : integer. on entry m must specify the number of data points.
		c           m > k-max(ib-1,0)-max(ie-1,0). unchanged on exit.
		c   u     : real array of dimension at least (m). before entry,
		c           u(i) must be set to the i-th value of the parameter variable
		c           u for i=1,2,...,m. these values must be supplied in
		c           strictly ascending order and will be unchanged on exit.
		c   mx    : integer. on entry mx must specify the actual dimension of
		c           the arrays x and xx as declared in the calling (sub)program
		c           mx must not be too small (see x). unchanged on exit.
		c   x     : real array of dimension at least idim*m.
		c           before entry, x(idim*(i-1)+j) must contain the j-th coord-
		c           inate of the i-th data point for i=1,2,...,m and j=1,2,...,
		c           idim. unchanged on exit.
		c   xx    : real array of dimension at least idim*m.
		c           used as working space. on exit xx contains the coordinates
		c           of the data points to which a spline curve with zero deriv-
		c           ative constraints has been determined.
		c           if the computation mode iopt =1 is used xx should be left
		c           unchanged between calls.
		c   w     : real array of dimension at least (m). before entry, w(i)
		c           must be set to the i-th value in the set of weights. the
		c           w(i) must be strictly positive. unchanged on exit.
		c           see also further comments.
		c   ib    : integer. on entry ib must specify the number of derivative
		c           constraints for the curve at the begin point. 0<=ib<=(k+1)/2
		c           unchanged on exit.
		c   db    : real array of dimension nb. before entry db(idim*l+j) must
		c           contain the l-th order derivative of sj(u) at u=u(1) for
		c           j=1,2,...,idim and l=0,1,...,ib-1 (if ib>0).
		c           unchanged on exit.
		c   nb    : integer, specifying the dimension of db. nb>=max(1,idim*ib)
		c           unchanged on exit.
		c   ie    : integer. on entry ie must specify the number of derivative
		c           constraints for the curve at the end point. 0<=ie<=(k+1)/2
		c           unchanged on exit.
		c   de    : real array of dimension ne. before entry de(idim*l+j) must
		c           contain the l-th order derivative of sj(u) at u=u(m) for
		c           j=1,2,...,idim and l=0,1,...,ie-1 (if ie>0).
		c           unchanged on exit.
		c   ne    : integer, specifying the dimension of de. ne>=max(1,idim*ie)
		c           unchanged on exit.
		c   k     : integer. on entry k must specify the degree of the splines.
		c           k=1,3 or 5.
		c           unchanged on exit.
		c   s     : real.on entry (in case iopt>=0) s must specify the smoothing
		c           factor. s >=0. unchanged on exit.
		c           for advice on the choice of s see further comments.
		c   nest  : integer. on entry nest must contain an over-estimate of the
		c           total number of knots of the splines returned, to indicate
		c           the storage space available to the routine. nest >=2*k+2.
		c           in most practical situation nest=m/2 will be sufficient.
		c           always large enough is nest=m+k+1+max(0,ib-1)+max(0,ie-1),
		c           the number of knots needed for interpolation (s=0).
		c           unchanged on exit.
		c   n     : integer.
		c           unless ier = 10 (in case iopt >=0), n will contain the
		c           total number of knots of the smoothing spline curve returned
		c           if the computation mode iopt=1 is used this value of n
		c           should be left unchanged between subsequent calls.
		c           in case iopt=-1, the value of n must be specified on entry.
		c   t     : real array of dimension at least (nest).
		c           on successful exit, this array will contain the knots of the
		c           spline curve,i.e. the position of the interior knots t(k+2),
		c           t(k+3),..,t(n-k-1) as well as the position of the additional
		c           t(1)=t(2)=...=t(k+1)=ub and t(n-k)=...=t(n)=ue needed for
		c           the b-spline representation.
		c           if the computation mode iopt=1 is used, the values of t(1),
		c           t(2),...,t(n) should be left unchanged between subsequent
		c           calls. if the computation mode iopt=-1 is used, the values
		c           t(k+2),...,t(n-k-1) must be supplied by the user, before
		c           entry. see also the restrictions (ier=10).
		c   nc    : integer. on entry nc must specify the actual dimension of
		c           the array c as declared in the calling (sub)program. nc
		c           must not be too small (see c). unchanged on exit.
		c   c     : real array of dimension at least (nest*idim).
		c           on successful exit, this array will contain the coefficients
		c           in the b-spline representation of the spline curve s(u),i.e.
		c           the b-spline coefficients of the spline sj(u) will be given
		c           in c(n*(j-1)+i),i=1,2,...,n-k-1 for j=1,2,...,idim.
		c   cp    : real array of dimension at least 2*(k+1)*idim.
		c           on exit cp will contain the b-spline coefficients of a
		c           polynomial curve which satisfies the boundary constraints.
		c           if the computation mode iopt =1 is used cp should be left
		c           unchanged between calls.
		c   np    : integer. on entry np must specify the actual dimension of
		c           the array cp as declared in the calling (sub)program. np
		c           must not be too small (see cp). unchanged on exit.
		c   fp    : real. unless ier = 10, fp contains the weighted sum of
		c           squared residuals of the spline curve returned.
		c   wrk   : real array of dimension at least m*(k+1)+nest*(6+idim+3*k).
		c           used as working space. if the computation mode iopt=1 is
		c           used, the values wrk(1),...,wrk(n) should be left unchanged
		c           between subsequent calls.
		c   lwrk  : integer. on entry,lwrk must specify the actual dimension of
		c           the array wrk as declared in the calling (sub)program. lwrk
		c           must not be too small (see wrk). unchanged on exit.
		c   iwrk  : integer array of dimension at least (nest).
		c           used as working space. if the computation mode iopt=1 is
		c           used,the values iwrk(1),...,iwrk(n) should be left unchanged
		c           between subsequent calls.
		c   ier   : integer. unless the routine detects an error, ier contains a
		c           non-positive value on exit, i.e.
	 *
	 */
	void concur_(int* iopt, int* idim, int* m, double* u, int* mx, double* x, double* xx, double* w,
			int* ib, double* db, int* nb, int* ie, double* de, int* ne,
			int* k, double* s, int* nest, int* n, double* t, int* nc, double* c,
			int* np, double* cp, double* fp, double* wrk, int* lwrk, int* iwrk, int* ier);

	/*
		  input parameters:
		c    t    : array,length n, which contains the position of the knots.
		c    n    : integer, giving the total number of knots of s(x).
		c    c    : array,length n, which contains the b-spline coefficients.
		c    k    : integer, giving the degree of s(x).
		c    x    : array,length m, which contains the points where s(x) must
		c           be evaluated.
		c    m    : integer, giving the number of points where s(x) must be
		c           evaluated.
		c    e    : integer, if 0 the spline is extrapolated from the end
		c           spans for points not in the support, if 1 the spline
		c           evaluates to zero for those points, if 2 ier is set to
		c           1 and the subroutine returns, and if 3 the spline evaluates
		c           to the value of the nearest boundary point.
		c
		c  output parameter:
		c    y    : array,length m, giving the value of s(x) at the different
		c           points.
		c    ier  : error flag
		c      ier = 0 : normal return
		c      ier = 1 : argument out of bounds and e == 2
		c      ier =10 : invalid input data (see restrictions)
	 */
	void splev_(double* t, int* n, double* c, int* k, double* x, double* y, int* m, int* e, int* ier);

	/*
		c  calling sequence:
		c     call splder(t,n,c,k,nu,x,y,m,e,wrk,ier)
		c
		c  input parameters:
		c    t    : array,length n, which contains the position of the knots.
		c    n    : integer, giving the total number of knots of s(x).
		c    c    : array,length n, which contains the b-spline coefficients.
		c    k    : integer, giving the degree of s(x).
		c    nu   : integer, specifying the order of the derivative. 0<=nu<=k
		c    x    : array,length m, which contains the points where the deriv-
		c           ative of s(x) must be evaluated.
		c    m    : integer, giving the number of points where the derivative
		c           of s(x) must be evaluated
		c    e    : integer, if 0 the spline is extrapolated from the end
		c           spans for points not in the support, if 1 the spline
		c           evaluates to zero for those points, and if 2 ier is set to
		c           1 and the subroutine returns.
		c    wrk  : real array of dimension n. used as working space.
		c
		c  output parameters:
		c    y    : array,length m, giving the value of the derivative of s(x)
		c           at the different points.
		c    ier  : error flag
		c      ier = 0 : normal return
		c      ier = 1 : argument out of bounds and e == 2
		c      ier =10 : invalid input data (see restrictions)
	 */
	void splder_(double* t, int* n, double* c, int* k, int* nu, double* x, double* y, int* m, int* e, double* wrk, int* ier);

}


namespace uav {
namespace math {

/**
 * Cubic smoothing spline with end constraints.
 */
template <class P>
class SmoothSpline {
private:

	std::vector<double> m_t;					///<! Knots
	std::vector<double> m_c;					///<! Coefficients

	size_t m_xidx;								///<! Index for getting the x coordinate from the point object.
	size_t m_yidx;								///<! Index for getting the y coordinate from the point object.

	int m_k; 									///<! Degree of spline function.
	int m_ier;									///<! Error result.
	double m_resid;								///<! The sum of squared residuals.

	std::string m_errStr;						///<! Descriptive message about the most recent error.

public:

	/**
	 * Create a smoothing spline of the given order (default, 3).
	 *
	 * @param order The order of the spline's polynomial. Defaults to 3.
	 * @param xidx The index into each point for the x coordinate (abscissa).
	 * @param yidx The index into each point for the y coordinate (ordinate).
	 */
	SmoothSpline(int order = 3, size_t xidx = 0, size_t yidx = 1) :
		m_xidx(xidx),
		m_yidx(yidx),
		m_k(order),
		m_ier(NOT_RUN),
		m_resid(0) {}

	void setXIndex(size_t xidx) {
		m_xidx = xidx;
	}

	size_t xIndex() const {
		return m_xidx;
	}

	void setYIndex(size_t yidx) {
		m_yidx = yidx;
	}

	size_t yIndex() const {
		return m_yidx;
	}

	/**
	 * Set the order, which will be from 1 (linear) to 5 (quintic).
	 *
	 * @param order The order of each polynomial function.
	 */
	void setOrder(int order) {
		m_k = order;
	}

	/**
	 * Get the order, which will be from 1 (linear) to 5 (quintic).
	 *
	 * @return The order of each polynomial function.
	 */
	int order() const {
		return m_k;
	}

	/**
	 * Get the sum of squared residuals from the last fit.
	 *
	 * @return The sum of squared residuals from the last fit.
	 */
	double residual() const {
		return m_resid;
	}

	/**
	 * Get the error result.
	 *
	 * @return The error result.
	 */
	int error() const {
		return m_ier;
	}

	/**
	 * Get the error message.
	 *
	 * @return The error message.
	 */
	const std::string& errorStr() const {
		return m_errStr;
	}

	/**
	 * Returns true if the most recent call to fit was successful.
	 */
	bool valid() const {
		return m_ier <= 0;
	}

	const std::vector<double>& knots() const {
		return m_t;
	}

	/**
	 * @param pts 		A list of points with an x and y property. X is the abscissa; y is the ordinate.
	 * @param weight 	A scalar giving the weight for each data point.
	 * @param s 		The smoothing factor.
	 * @param bc		A list of constraints, corresponding to the ith derivative at the beginning of the spline. Length between 0-k.
	 * @param ec		A list of constraints, corresponding to the ith derivative at the beginning of the spline. Length between 0-k.
	 */
	bool fit(const std::vector<P>& pts, double weight, double s,
			const std::vector<double>& bc = {}, const std::vector<double>& ec = {}) {

		std::vector<double> weights(pts.size());
		for(size_t i = 0; i < weights.size(); ++i)
			weights[i] = weight;

		return fit(pts, weights, s, bc, ec);
	}

	/**
	 * @param pts 		A list of points with an x and y property. X is the abscissa; y is the ordinate.
	 * @param weights	A list of weights at each data point.
	 * @param s 		The smoothing factor.
	 * @param bc		A list of constraints, corresponding to the ith derivative at the beginning of the spline. Length between 0-k.
	 * @param ec		A list of constraints, corresponding to the ith derivative at the beginning of the spline. Length between 0-k.
	 */
	bool fit(const std::vector<P>& pts, const std::vector<double>& weights, double s,
			const std::vector<double>& bc = {}, const std::vector<double>& ec = {}) {

		m_errStr = "";
		m_ier = NOT_RUN;

		if(pts.size() < m_k - std::max((int) bc.size() - 1, 0) - std::max((int) ec.size() - 1, 0)) {
			m_errStr = "Too few points.";
			return false;
		}

		if(weights.size() != pts.size()) {
			m_errStr = "Weights and coordinates must be the same size.";
			return false;
		}

		static std::vector<double> u; 		// ? If iopt is zero, need not be supplied.
		static std::vector<double> x; 		// Coordinates (multi dimensional).
		static std::vector<double> w; 		// Weights
		static std::vector<double> db;		// Constains the constraints on the derivatives from 0->n at beginning.
		static std::vector<double> de;		// Constains the constraints on the derivatives from 0->n at end.
		static std::vector<double> cp;		// Coefficients with boundary constraints.
		static std::vector<double> xx;		// Working space.
		static std::vector<double> wrk;		// Work space.
		static std::vector<int> iwrk;		// Work space.

		int ib = bc.size();			// The number of derivative constraints at beginning. Max: (k + 1) / 2
		int ie = ec.size();			// The number of derivative constraints at end. Max: (k + 1) / 2
		int m = pts.size();			// Number of points.

		int iopt = 0;				// Smooth spline -- no previous run.
		int idim = 1;				// Number of dimensions.
		int k = m_k;				// Degree of spline.

		int mx = m * idim;			// Dimension of x array.
		int nest = m + k + 1;		// Estimate of number of knots for storage allotment.
		int nc = nest * idim;		// Dimension of coefficient array.

		int nb = std::max(1, ib * idim);					// Dimension of db.
		int ne = std::max(1, ie * idim);					// Dimension of de.
		int np = 2 * (k + 1) * idim;							// Number of coefficients with boundary constraints.
		int lwrk = m * (k + 1) + nest * (6 + idim + 3 * k);		// Size of work space.

		int n;	// Number of resulting knots.

		// Initalize the arrays.
		u.resize(m);
		w.resize(m);
		x.resize(mx);
		xx.resize(mx);
		wrk.resize(lwrk);
		iwrk.resize(nest);
		db.resize(nb);
		de.resize(ne);
		cp.resize(np);
		m_t.resize(nest);
		m_c.resize(nc);

		// Populate the points buffer.
		for(size_t i = 0; i < pts.size(); ++i) {
			u[i] = pts[i][m_xidx];
			x[i] = pts[i][m_yidx];
			w[i] = weights[i];
		}

		// Populate the constraints buffers.
		if(ib > 0) {
			for(int l = 0; l < ib; ++l) {
				for(int j = 0; j < idim; ++j)
					db[idim * l + j] = bc[l];
			}
		}
		if(ie > 0) {
			for(int l = 0; l < ie; ++l) {
				for(int j = 0; j < idim; ++j)
					de[idim * l + j] = ec[l];
			}
		}

		concur_(&iopt, &idim, &m, u.data(), &mx, x.data(), xx.data(), w.data(),
				&ib, db.data(), &nb, &ie, de.data(), &ne,
				&k, &s, &nest, &n, m_t.data(), &nc, m_c.data(),
				&np, cp.data(), &m_resid, wrk.data(), &lwrk, iwrk.data(), &m_ier);

		// Trim the coef and knot arrays.
		if(nc < m_c.size())
			m_c.resize(nc);
		if(n < m_t.size())
			m_t.resize(n);

		if(m_ier != 0)
			std::cerr << "Error: " << m_ier << "\n";

		switch(m_ier) {
		case -1:
			m_errStr = "Interpolating spline.";
			break;
		case -2:
			m_errStr = "Least squares spline.";
			break;
		case 1:
			m_errStr = "nest is too small.";
			break;
		case 2:
			m_errStr = "s is probably too small.";
			break;
		case 3:
			m_errStr = "Maximum number of iterations exceeded. s may be too small";
			break;
		case 4:
			m_errStr = "x values are not strictly increasing.";
			break;
		case 5:
			m_errStr = "d value must be positive.";
			break;
		case 10:
			if(ib > (m_k + 1) / 2 || ie > (m_k + 1) / 2)
				m_errStr = "Constraint array is too large.";
			if(iopt < -1 || iopt > 1)
				m_errStr = "iopt must be in the range -1 - 1.";
			if(m_k < 1 || m_k > 5 || m_k % 2 == 0)
				m_errStr = "Degree must be odd, in the range 1 - 5.";
			if(m <= m_k)
				m_errStr = "Number of coordinates must be larger than the degree.";
			if(nest <= 2 * m_k + 2)
				m_errStr = "Nest must be larger than 2 * k + 2.";
			if(weights.size() != pts.size())
				m_errStr = "Number of weights must equal number of coordinates.";
			if(lwrk < (m_k + 1) * m + nest * (7 + 3 * m_k))
				m_errStr = "lwrk is too small.";
			if(iopt == -1 && ((n < 2 * m_k + 2) || (n > std::min(nest, m + m_k + 1))))
				m_errStr = "n is out of range.";
			break;
		}

		return m_ier == 0;
	}

	/**
	 * Return a list of points representing the knots and their associated
	 * spline values (0-th derivative).
	 *
	 * @param kts A list of points.
	 * @return True if successful.
	 */
	bool knots(std::vector<P>& kts) {
		std::vector<double> y;
		if(evaluate(m_t, y, 0)) {
			kts.clear();
			for(size_t i = 0; i < m_t.size(); ++i)
				kts.emplace_back(0, m_t[i], y[i]);
			return true;
		}
		return false;
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
		 int m = x.size();
		 int n = m_t.size();
		 int e = 3; // No extrapolate
		 int ier = NOT_RUN;
		 y.resize(x.size());
		 if(derivative == 0) {
			 splev_(m_t.data(), &n, m_c.data(), &m_k, (double*) x.data(), y.data(), &m, &e, &ier);
		 } else {
			 std::vector<double> wrk(n);
			 splder_(m_t.data(), &n, m_c.data(), &m_k, &derivative, (double*) x.data(), y.data(), &m, &e, wrk.data(), &ier);
		 }
		 return ier == 0;
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
	 * Populate the vector with regularly spaced doubles, according to count.
	 *
	 * @param x0 The starting x.
	 * @param x1 The ending x.
	 * @param lst The output list of values.
	 * @param count The number of items.
	 */
	void linspace(double x0, double x1, std::vector<double>& lst, int count) {
		lst.clear();
		if(count <= 2) {
			lst.push_back(x0);
			lst.push_back(x1);
		} else {
			lst.resize(count);
			double dist = (x1 - x0) / (count - 1);
			for(size_t i = 0; i < count - 1; ++i)
				lst[i] = x0 + dist * i;
			lst[count - 1] = x1;
		}
	}};

} // math
} // uav


#endif /* INCLUDE_MATH_SMOOTHSPLINE_HPP_ */
