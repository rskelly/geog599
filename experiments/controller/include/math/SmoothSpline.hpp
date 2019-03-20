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

	/**
	c  from  * a practical guide to splines *  by c. de boor
	calls  setupq, chol1d
	c
	c  constructs the cubic smoothing spline  f  to given data  (x(i),y(i)),
	c  i=1,...,npoint, which has as small a second derivative as possible
	c  while
	c  s(f) = sum( ((y(i)-f(x(i)))/dy(i))**2 , i=1,...,npoint ) .le. s .
	c
	c******  i n p u t  ******
	c  x(1),...,x(npoint)   data abscissae,  a s s u m e d  to be strictly
	c        increasing .
	c  y(1),...,y(npoint)     corresponding data ordinates .
	c  dy(1),...,dy(npoint)     estimate of uncertainty in data,  a s s u m-
	c        e d  to be positive .
	c  npoint.....number of data points,  a s s u m e d  .gt. 1
	c  s.....upper bound on the discrete weighted mean square distance of
	c        the approximation  f  from the data .
	c
	c******  w o r k  a r r a y s  *****
	c  v.....of size (npoint,7)
	c  a.....of size (npoint,4)
	c
	c*****  o u t p u t  *****
	c  a(.,1).....contains the sequence of smoothed ordinates .
	c  a(i,j) = f^(j-1)(x(i)), j=2,3,4, i=1,...,npoint-1 ,  i.e., the
	c        first three derivatives of the smoothing spline  f  at the
	c        left end of each of the data intervals .
	c     w a r n i n g . . .   a  would have to be transposed before it
	c        could be used in  ppvalu .
	c
	c******  m e t h o d  ******
	c     The matrices  Q-transp*d  and  Q-transp*D**2*Q  are constructed in
	c   s e t u p q  from  x  and  dy , as is the vector  qty = Q-transp*y .
	c  Then, for given  p , the vector  u  is determined in  c h o l 1 d  as
	c  the solution of the linear system
	c               (6(1-p)Q-transp*D**2*Q + p*Q)u  = qty  .
	c  From  u , the smoothing spline  f  (for this choice of smoothing par-
	c  ameter  p ) is obtained in the sense that
	c                        f(x(.))  =  y - 6(1-p)D**2*Q*u        and
	c                      f''(x(.))  =  6*p*u                      .
	c     The smoothing parameter  p  is found (if possible) so that
	c                sf(p)  =  s ,
	c  with  sf(p) = s(f) , where  f  is the smoothing spline as it depends
	c  on  p .  if  s = 0, then p = 1 . if  sf(0) .le. s , then p = 0 .
	c  Otherwise, the secant method is used to locate an appropriate  p  in
	c  the open interval  (0,1) . However, straightforward application of
	c  the secant method, as done in the original version of this program,
	c  can be very slow and is influenced by the units in which  x  and  y
	c  are measured, as C. Reinsch has pointed out. Instead, on recommend-
	c  ation from C. Reinsch, the secant method is applied to the function
	c           g:q |--> 1/sqrt{sfq(q)} - 1/sqrt{s} ,
	c  with  sfq(q) := sf(q/(1+q)), since  1/sqrt{sfq}  is monotone increasing
	c  and close to linear for larger  q . One starts at  q = 0  with a
	c  Newton step, i.e.,
	c                q_0 = 0,  q_1 = -g(0)/g'(0)
	c  with  g'(0) = -(1/2) sfq(0)^{-3/2} dsfq, where dsfq = -12*u-transp*r*u ,
	c  and  u  as obtained for  p = 0 . Iteration terminates as soon as
	c   abs(sf - s) .le. .01*s .
	c
	c     logical test
	c     parameter (test = .true.)
	c     integer itercnt
	 */
	void smooth_(float* x, float* y, float* dy, int* npoint, float* s, float* v, float* a);

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

/**
 * Cubic smoothing spline: S(x)
 * - 2N (6) degrees of freedom.
 * - We require that
 * 	- S(x1) = S(x2),
 * 	- S'(x1) = S'(x2) and
 * 	- S''(x1) = S'(x2).
 * - 2 criteria:
 * 	- Passes as near to ordinates as possible.
 * 	- As smooth as possible.
 * 	- S(x) = p * sum((f(x) -
 *
 *
 *
 */
template <class P>
class SmoothSpline {
private:
	std::vector<double> m_x; 					// Abscissae
	std::vector<double> m_y; 					// Ordinates
	std::vector<double> m_w; 					// Weights

	std::vector<double> m_t;					// Knots
	std::vector<double> m_c;					// Coefficients

	size_t m_xidx;								// Indices for getting coordinates from the P object.
	size_t m_yidx;

	int m_k; 									// Degree
	double m_knotDist;							// The distance between knots, if equidistant knots are required.
	bool m_validFit;							// True if a fit has completed successfully.

	/**
	 * Populate the vector with regularly spaced doubles, according to knotDistance,
	 * starting with x0, ending with x1. The spacing between x1 and lst[-2] may be less
	 * than knotDistance. Padded with k-1 repeats at each end (where k is the order
	 *
	 * @param x0 The starting x.
	 * @param x1 The ending x.
	 * @param lst The output list of values.
	 * @returns The number items added to the list.
	 */
	int linspace(double x0, double x1, std::vector<double>& lst) {
		if(m_knotDist == 0)
			return -1;

		lst.clear();

		for(int i = 0; i < m_k; ++i)
			lst.push_back(x0);
		for(double x = x0; x < x1; x += m_knotDist)
			lst.push_back(x);
		for(int i = 0; i < m_k + 1; ++i)
			lst.push_back(x1);
		return lst.size();
	}

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
		m_validFit(false),
		m_knotDist(0) {}

	void setKnotDistance(double dist) {
		m_knotDist = dist;
	}

	double knotDistance() const {
		return m_knotDist;
	}

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
	 * Returns true if the most recent call to fit was successful.
	 */
	bool valid() const {
		return m_validFit;
	}

	const std::vector<double>& knots() const {
		return m_t;
	}

	/**
	 * @param pts 		A list of points with an x and y property. X is the abscissa; y is the ordinate.
	 * @param weight 	A scalar giving the weight for each data point.
	 * @param s 		The smoothing factor.
	 * @param out 		The output of the function; a list of "curve" objects containing the pair
	 * 					of knots and the second derivative at each.
	 */
	bool fit(const std::vector<P>& pts, double weight, double s) {

		std::vector<double> weights(pts.size());
		for(size_t i = 0; i < weights.size(); ++i)
			weights[i] = weight;

		return fit(pts, weights, s);
	}

	/**
	 * @param pts 		A list of points with an x and y property. X is the abscissa; y is the ordinate.
	 * @param weights	A list of weights at each data point.
	 * @param s 		The smoothing factor.
	 */
	bool fit(const std::vector<P>& pts, const std::vector<double>& weights, double s) {

		int minn = 2 * m_k + 2; // The minimum number of points. Why?

		if(pts.size() != weights.size()) {
			std::cerr << "Weights and points must have the same length\n";
			return false;
		}

		if(pts.size() < minn || pts.size() != weights.size()) {
			std::cerr << "Too few points (" << pts.size() << " of " << minn << ")\n";
			return false;
		}

		// Inputs
		int iopt = 1;					// Determines how s is calculated.
		int m = (int) pts.size();		// Number of points.
		m_x.resize(m); 					// Abscissae
		m_y.resize(m); 					// Ordinates
		m_w.resize(m); 					// Weights
		double tol = 0.0001;			// Epsilon

		int k1 = m_k + 1;				// ?
		int k2 = m_k + 2;;				// ?
		int nest = m + m_k + 1; 		// Estimate for n.
		double fp;						// ?
		int maxit = 100;					// Maximum number of iterations.

		// Outputs
		int ierr = 0;					// Error return.
		int n = -1;						// Number of knots.

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
			m_x[i] = pts[i][m_xidx];
			m_y[i] = pts[i][m_yidx];
			m_w[i] = weights[i];
		}

		// First and last ordinates.
		double xb = m_x[0];
		double xe = m_x[m_x.size() - 1];

		std::cout << "1 nest: " << nest << ", n: " << n << "\n";

		fpcurf_(&iopt, m_x.data(), m_y.data(), m_w.data(), &m, &xb, &xe,
			&m_k, &s, &nest, &tol, &maxit, &k1, &k2, &n,
			m_t.data(), m_c.data(), &fp, fpint.data(), z.data(), a.data(), b.data(),
			g.data(), q.data(), nrdata.data(), &ierr);

		std::cout << "2 nest: " << nest << ", n: " << n << "\n";

		// Trim the result arrays.
		if(n != m_t.size()) {
			m_t.resize(n);
			m_c.resize(n);
		}

		m_validFit = ierr == 0 || ierr == -2;

		if(ierr != 0)
			std::cerr << "Error: " << ierr << "\n";

		switch(ierr) {
		case 1:
			throw std::runtime_error("nest is too small.");
		case 2:
			throw std::runtime_error("s must be positive.");
		case 3:
			throw std::runtime_error("Maximum number of iterations exceeded."); //eps must be positive and <= 1.");
		case 4:
			throw std::runtime_error("x values are not strictly increasing.");
		case 5:
			throw std::runtime_error("d value must be positive.");
		}

		return true;
	}

	bool fitDeBoor(const std::vector<P>& pts, double weight, float s) {
		std::vector<double> weights(pts.size());
		std::fill(weights.begin(), weights.end(), weight);
		return fitDeBoor(pts, weights, s);
	}

	bool fitDeBoor(const std::vector<P>& pts, const std::vector<double>& weights, float s) {

		if(pts.size() != weights.size()) {
			std::cerr << "Weights and points must have the same length\n";
			return false;
		}

		// Inputs
		int m = (int) pts.size();		// Number of points.
		std::vector<float> x(m);		// Abscissae
		std::vector<float> y(m);		// Ordinates
		std::vector<float> dy(m); 		// Weights

		// Outputs
		std::vector<float> a(m * 4);
		std::vector<float> v(m * 7);

		// Copy the values.
		for(int i = 0; i < m; ++i) {
			x[i] = pts[i][m_xidx];
			y[i] = pts[i][m_yidx];
			dy[i] = weights[i];
		}

		smooth_(x.data(), y.data(), dy.data(), &m, &s, v.data(), a.data());

		return true;
	}

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
	 * @param x The x-coordinates.
	 * @param y The y-coordinates (output).
	 * @param derivative The derivative to evaluate. Defaults to zero, the original function.
	 */
	bool evaluate(const std::vector<double>& x, std::vector<double>& y, int derivative = 0) {
		if(!m_validFit)
			return false;
		 int m = x.size();
		 int n = m_t.size();
		 int e = 1; // No extrapolate
		 int ier = 0;
		 y.resize(x.size());
		 if(derivative == 0) {
			 splev_(m_t.data(), &n, m_c.data(), &m_k, (double*) x.data(), y.data(), &m, &e, &ier);
		 } else if(derivative > 0 && derivative <= 3) {
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
		if(!m_validFit || x < m_t[0] || x > m_t[m_t.size() - 1])
			return false;
		std::vector<double> xx(1), yy(1);
		xx[0] = x;
		if(evaluate(xx, yy, derivative)) {
			y = yy[0];
			return true;
		}
		return false;
	}

};

} // math
} // uav


#endif /* INCLUDE_MATH_SMOOTHSPLINE_HPP_ */
