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


extern "C" {

	/*
	 * Calls in to the fpcurf function of the fitpack (fortran) library.
	 *
	 * Comments from source file:
		c  ..
		c  ..scalar arguments..
			  real*8 xb,xe,s,tol,fp
			  integer iopt,m,k,nest,maxit,k1,k2,n,ier
		c  ..array arguments..
			  real*8 x(m),y(m),w(m),t(nest),c(nest),fpint(nest),
			 * z(nest),a(nest,k1),b(nest,k2),g(nest,k2),q(m,k1)
			  integer nrdata(nest)
		c  ..local scalars..
			  real*8 acc,con1,con4,con9,cos,half,fpart,fpms,fpold,fp0,f1,f2,f3,
			 * one,p,pinv,piv,p1,p2,p3,rn,sin,store,term,wi,xi,yi
			  integer i,ich1,ich3,it,iter,i1,i2,i3,j,k3,l,l0,
			 * mk1,new,nk1,nmax,nmin,nplus,npl1,nrint,n8
		c  ..local arrays..
			  real*8 h(7)
		c  ..function references
			  real*8 abs,fprati
			  integer max0,min0
		c  ..subroutine references..
		c    fpback,fpbspl,fpgivs,fpdisc,fpknot,fprota
		c  ..

		To determine the value of iopt:

		cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
		c  part 1: determination of the number of knots and their position     c
		c  **************************************************************      c
		c  given a set of knots we compute the least-squares spline sinf(x),   c
		c  and the corresponding sum of squared residuals fp=f(p=inf).         c
		c  if iopt=-1 sinf(x) is the requested approximation.                  c
		c  if iopt=0 or iopt=1 we check whether we can accept the knots:       c
		c    if fp <=s we will continue with the current set of knots.         c
		c    if fp > s we will increase the number of knots and compute the    c
		c       corresponding least-squares spline until finally fp<=s.        c
		c    the initial choice of knots depends on the value of s and iopt.   c
		c    if s=0 we have spline interpolation; in that case the number of   c
		c    knots equals nmax = m+k+1.                                        c
		c    if s > 0 and                                                      c
		c      iopt=0 we first compute the least-squares polynomial of         c
		c      degree k; n = nmin = 2*k+2                                      c
		c      iopt=1 we start with the set of knots found at the last         c
		c      call of the routine, except for the case that s > fp0; then     c
		c      we compute directly the least-squares polynomial of degree k.   c
		cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc


		cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
		c  part 2: determination of the smoothing spline sp(x).                c
		c  ***************************************************                 c
		c  we have determined the number of knots and their position.          c
		c  we now compute the b-spline coefficients of the smoothing spline    c
		c  sp(x). the observation matrix a is extended by the rows of matrix   c
		c  b expressing that the kth derivative discontinuities of sp(x) at    c
		c  the interior knots t(k+2),...t(n-k-1) must be zero. the corres-     c
		c  ponding weights of these additional rows are set to 1/p.            c
		c  iteratively we then have to determine the value of p such that      c
		c  f(p)=sum((w(i)*(y(i)-sp(x(i))))**2) be = s. we already know that    c
		c  the least-squares kth degree polynomial corresponds to p=0, and     c
		c  that the least-squares spline corresponds to p=infinity. the        c
		c  iteration process which is proposed here, makes use of rational     c
		c  interpolation. since f(p) is a convex and strictly decreasing       c
		c  function of p, it can be approximated by a rational function        c
		c  r(p) = (u*p+v)/(p+w). three values of p(p1,p2,p3) with correspond-  c
		c  ing values of f(p) (f1=f(p1)-s,f2=f(p2)-s,f3=f(p3)-s) are used      c
		c  to calculate the new value of p such that r(p)=s. convergence is    c
		c  guaranteed by taking f1>0 and f3<0.                                 c
		cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc

		@param iopt 		An integer which determines the number of knots. (-1, 0 or 1).
		@param x			The list of abscissae. (Array (m))
		@param y			The list of ordinates. (Array (m))
		@param w			The list of weights. Default 1.0. (Array (m))
		@param m			The number of coordinates.
		@param xb			The value of the first element in x. (Overridable? Why?)
		@param xe			The value of the last element in x.
		@param k			The degree of the spline (1 >= k >= 5).
		@param s			The smoothing parameter. Default m. (>= 0).
		@param nest			Estimated length of output; used for working buffers. (s == 0.0 ? m + k + 1 : MAX(m / 2 , 2 * k1))
		@param tol			Tolerance. Defualt 0.001. (Epsilon?)
		@param maxit		The maximum number of iterations. Default 20.
		@param k1			No idea. k1 = k + 1
		@param k2			No idea. k2 = k + 2
		@param n			The number of output knots.
		@param t			(Array (nest))
		@param c			(Array (nest))
		@param fp			The residual. (Compare to s)
		@param fpint		(Array (nest))
		@param z			(Array (nest))
		@param a			(Array (nest))
		@param b			(Array (nest))
		@param g			(Array (nest))
		@param q			(Array (m))
		@param nrdata		(Array (nest))
		@param ier			Output error value.
	*/
	void fpcurf_(int* iopt, double* x, double* y, double* w, int* m, double* xb, double* xe,
			int* k, double* s, int* nest, double* tol, int* maxit, int* k1, int* k2, int* n,
			double* t, double* c, double* fp, double* fpint, double* z, double* a, double* b,
			double* g, double* q, int* nrdata, int* ier);

	/*
	subroutine splev(t,n,c,k,x,y,m,e,ier)
	c  subroutine splev evaluates in a number of points x(i),i=1,2,...,m
	c  a spline s(x) of degree k, given in its b-spline representation.
	c
	c  calling sequence:
	c     call splev(t,n,c,k,x,y,m,e,ier)
	c
	c  input parameters:
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
	c
	c  restrictions:
	c    m >= 1
	c--    t(k+1) <= x(i) <= x(i+1) <= t(n-k) , i=1,2,...,m-1.
	c
	c  other subroutines required: fpbspl.
	c
	c  references :
	c    de boor c  : on calculating with b-splines, j. approximation theory
	c                 6 (1972) 50-62.
	c    cox m.g.   : the numerical evaluation of b-splines, j. inst. maths
	c                 applics 10 (1972) 134-149.
	c    dierckx p. : curve and surface fitting with splines, monographs on
	c                 numerical analysis, oxford university press, 1993.
	c
	c  author :
	c    p.dierckx
	c    dept. computer science, k.u.leuven
	c    celestijnenlaan 200a, b-3001 heverlee, belgium.
	c    e-mail : Paul.Dierckx@cs.kuleuven.ac.be
	c
	c  latest update : march 1987
	c
	c++ pearu: 11 aug 2003
	c++   - disabled cliping x values to interval [min(t),max(t)]
	c++   - removed the restriction of the orderness of x values
	c++   - fixed initialization of sp to double precision value
	 *
	 * @param t 	A list containing the abscissae of the knots.
	 * @param n 	The number of knots.
	 * @param c 	The b-spline coefficients.
	 * @param k 	The degree of the spline.
	 * @param x 	The x-coordinates for which to evaluate the spline.
	 * @param y 	The y-coordinates resulting from evaluation (output).
	 * @param m 	The number of evaluated points.
	 * @param e 	Extrapolation (1).
	 * @param ier 	The error result.
	 */
	void splev_(double* t, int* n, double* c, int* k, double* x, double* y, int* m, int* e, int *ier);

	/*
	 *      subroutine splder(t,n,c,k,nu,x,y,m,e,wrk,ier)
			  implicit none
		c  subroutine splder evaluates in a number of points x(i),i=1,2,...,m
		c  the derivative of order nu of a spline s(x) of degree k,given in
		c  its b-spline representation.
		c
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
		c
		c  restrictions:
		c    0 <= nu <= k
		c    m >= 1
		c    t(k+1) <= x(i) <= x(i+1) <= t(n-k) , i=1,2,...,m-1.
		c
		c  other subroutines required: fpbspl
		c
		c  references :
		c    de boor c : on calculating with b-splines, j. approximation theory
		c                6 (1972) 50-62.
		c    cox m.g.  : the numerical evaluation of b-splines, j. inst. maths
		c                applics 10 (1972) 134-149.
		c   dierckx p. : curve and surface fitting with splines, monographs on
		c                numerical analysis, oxford university press, 1993.
		c
		c  author :
		c    p.dierckx
		c    dept. computer science, k.u.leuven
		c    celestijnenlaan 200a, b-3001 heverlee, belgium.
		c    e-mail : Paul.Dierckx@cs.kuleuven.ac.be
		c
		c  latest update : march 1987
		c
		c++ pearu: 13 aug 20003
		c++   - disabled cliping x values to interval [min(t),max(t)]
		c++   - removed the restriction of the orderness of x values
		c++   - fixed initialization of sp to double precision value
	 *
	 * @param t 	A list containing the abscissae of the knots.
	 * @param n 	The number of knots.
	 * @param c 	The b-spline coefficients.
	 * @param k 	The degree of the spline.
	 * @param nu	The derivative (0 <= nu <= n).
	 * @param x 	The x-coordinates for which to evaluate the spline.
	 * @param y 	The y-coordinates resulting from evaluation (output).
	 * @param m 	The number of evaluated points.
	 * @param e 	Extrapolation (1).
	 * @param wrk	A work array of length n.
	 * @param ier 	The error result.

	 */
	void splder_(double* t, int* n, double* c, int* k, int* nu, double* x, double* y, int* m, int* e, double* wrk, int* ier);
}


namespace uav {
namespace math {

template <class P>
class SmoothSpline {
private:
	std::vector<double> m_x; 					// Abscissae
	std::vector<double> m_y; 					// Ordinates
	std::vector<double> m_w; 					// Weights

	std::vector<double> m_t;					// Knots
	std::vector<double> m_c;					// Coefficients

	int m_k; 									// Degree

public:

	/**
	 * Create a smoothing spline of the given order (default, 3).
	 *
	 * @param order The order of the spline's polynomial. Defaults to 3.
	 */
	SmoothSpline(int order = 3) :
		m_k(order) {}

	/**
	 * @param pts 		A list of points with an x and y property. X is the abscissa; y is the ordinate.
	 * @param weight 	A scalar giving the weight for each data point.
	 * @param s 		The smoothing factor.
	 * @param out 		The output of the function; a list of "curve" objects containing the pair
	 * 					of knots and the second derivative at each.
	 */
	void fit(const std::vector<P>& pts, double weight, double s) {

		std::vector<double> weights(pts.size());
		for(size_t i = 0; i < weights.size(); ++i)
			weights[i] = weight;

		fit(pts, weights, s);
	}

	/**
	 * @param pts 		A list of points with an x and y property. X is the abscissa; y is the ordinate.
	 * @param w		 	A list of weights at each data point.
	 * @param s 		The smoothing factor.
	 * @param out 		The output of the function; a list of "curve" objects containing the pair
	 * 					of knots and the second derivative at each.
	 */
	void fit(const std::vector<P>& pts, const std::vector<double>& weights, double s) {

		if(pts.size() < 2 || pts.size() != weights.size())
			throw std::runtime_error("Weights and points must have the same length and be more than 2.");

		// Inputs
		int iopt = 0;					// Determines how s is calculated.
		int m = (int) pts.size();		// Number of points.
		m_x.resize(m); 					// Abscissae
		m_y.resize(m); 					// Ordinates
		m_w.resize(m); 					// Weights
		double tol = 0.001;				// Epsilon

		// Outputs
		int ierr = 0;					// Error return.
		int n = -1;						// Number of knots.

		int k1 = m_k + 1;				// ?
		int k2 = m_k + 2;;				// ?
		int nest = m + m_k + 1; 		// Estimate for n. s == 0.0 ? m + k + 1 : std::max(m / 2, 2 * k1);
		double fp;						// ?
		int maxit = 20;					// Maximum number of iterations.

		m_t.resize(nest);					// Knots
		m_c.resize(nest);					// Coefficients
		std::vector<double> fpint(nest);	// Temporary storage...
		std::vector<double> z(nest);
		std::vector<double> a(nest * k1);
		std::vector<double> b(nest * k2);
		std::vector<double> g(nest * k2);
		std::vector<double> q(m * k1);
		std::vector<int> nrdata(nest);

		// Copy the values.
		for(int i = 0; i < m; ++i) {
			m_x[i] = pts[i].x();
			m_y[i] = pts[i].y();
			m_w[i] = weights[i];
		}

		// First and last ordinates.
		double xb = m_x[0];
		double xe = m_x[m_x.size() - 1];

		fpcurf_(&iopt, m_x.data(), m_y.data(), m_w.data(), &m, &xb, &xe,
			&m_k, &s, &nest, &tol, &maxit, &k1, &k2, &n,
			m_t.data(), m_c.data(), &fp, fpint.data(), z.data(), a.data(), b.data(),
			g.data(), q.data(), nrdata.data(), &ierr);

		// Trim the result arrays.
		if(n > m_t.size()) {
			m_t.resize(n);
			m_c.resize(n);
		}

		/*
		switch(ierr) {
		case 1:
			throw std::runtime_error("nest is too small.");
		case 2:
			throw std::runtime_error("s must be positive.");
		case 3:
			throw std::runtime_error("eps must be positive and <= 1.");
		case 4:
			throw std::runtime_error("x values are not strictly increasing.");
		case 5:
			throw std::runtime_error("d value must be positive.");
		}
		*/

	}

	/**
	 * Evaluate the spline at the given positions in x for the given derivative (default 0).
	 * @param x The x-coordinates.
	 * @param y The y-coordinates (output).
	 * @param derivative The derivative to evaluate. Defaults to zero, the original function.
	 */
	void evaluate(const std::vector<double>& x, std::vector<double>& y, int derivative = 0) {
		 int m = x.size();
		 int n = m_t.size();
		 int e = 1; // No extrapolate
		 int ier = 0;
		 if(derivative == 0) {
			 splev_(m_t.data(), &n, m_c.data(), &m_k, (double*) x.data(), y.data(), &m, &e, &ier);
		 } else if(derivative > 0 && derivative <= 3) {
			 std::vector<double> wrk(n);
			 splder_(m_t.data(), &n, m_c.data(), &m_k, &derivative, (double*) x.data(), y.data(), &m, &e, wrk.data(), &ier);
		 }
	}

	/**
	 * Evaluate the spline at the given position in x for the given derivative (default 0).
	 * @param x The x-coordinate.
	 * @param y The y-coordinate (output).
	 * @param derivative The derivative to evaluate. Defaults to zero, the original function.
	 */
	void evaluate(double x, double& y, int derivative = 0) {
		 int m = 1;
		 int n = m_t.size();
		 int e = 1; // No extrapolate
		 int ier = 0;
		 if(derivative == 0) {
			 splev_(m_t.data(), &n, m_c.data(), &m_k, &x, &y, &m, &e, &ier);
		 } else if(derivative > 0 && derivative <= 3) {
			 std::vector<double> wrk(n);
			 splder_(m_t.data(), &n, m_c.data(), &m_k, &derivative, &x, &y, &m, &e, wrk.data(), &ier);
		 }
	}

};

} // math
} // uav


#endif /* INCLUDE_MATH_SMOOTHSPLINE_HPP_ */
