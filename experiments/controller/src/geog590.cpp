/*
 * geog590.cpp
 *
 *  Created on: Apr 1, 2019
 *      Author: rob
 */

#include <vector>
#include <iostream>

extern "C" {

	/*
	 *  parameters:
		c   iopt  : integer flag. on entry iopt must specify whether a weighted
		c           least-squares spline curve (iopt=-1) or a smoothing spline
		c           curve (iopt=0 or 1) must be determined.if iopt=0 the routine
		c           will start with an initial set of knots t(i)=ub,t(i+k+1)=ue,
		c           i=1,2,...,k+1. if iopt=1 the routine will continue with the
		c           knots found at the last call of the routine.
		c           attention: a call with iopt=1 must always be immediately
		c           preceded by another call with iopt=1 or iopt=0.
		c           unchanged on exit.
		c   ipar  : integer flag. on entry ipar must specify whether (ipar=1)
		c           the user will supply the parameter values u(i),ub and ue
		c           or whether (ipar=0) these values are to be calculated by
		c           parcur. unchanged on exit.
		c   idim  : integer. on entry idim must specify the dimension of the
		c           curve. 0 < idim < 11.
		c           unchanged on exit.
		c   m     : integer. on entry m must specify the number of data points.
		c           m > k. unchanged on exit.
		c   u     : real array of dimension at least (m). in case ipar=1,before
		c           entry, u(i) must be set to the i-th value of the parameter
		c           variable u for i=1,2,...,m. these values must then be
		c           supplied in strictly ascending order and will be unchanged
		c           on exit. in case ipar=0, on exit,array u will contain the
		c           values u(i) as determined by parcur.
		c   mx    : integer. on entry mx must specify the actual dimension of
		c           the array x as declared in the calling (sub)program. mx must
		c           not be too small (see x). unchanged on exit.
		c   x     : real array of dimension at least idim*m.
		c           before entry, x(idim*(i-1)+j) must contain the j-th coord-
		c           inate of the i-th data point for i=1,2,...,m and j=1,2,...,
		c           idim. unchanged on exit.
		c   w     : real array of dimension at least (m). before entry, w(i)
		c           must be set to the i-th value in the set of weights. the
		c           w(i) must be strictly positive. unchanged on exit.
		c           see also further comments.
		c   ub,ue : real values. on entry (in case ipar=1) ub and ue must
		c           contain the lower and upper bound for the parameter u.
		c           ub <=u(1), ue>= u(m). if ipar = 0 these values will
		c           automatically be set to 0 and 1 by parcur.
		c   k     : integer. on entry k must specify the degree of the splines.
		c           1<=k<=5. it is recommended to use cubic splines (k=3).
		c           the user is strongly dissuaded from choosing k even,together
		c           with a small s-value. unchanged on exit.
		c   s     : real.on entry (in case iopt>=0) s must specify the smoothing
		c           factor. s >=0. unchanged on exit.
		c           for advice on the choice of s see further comments.
		c   nest  : integer. on entry nest must contain an over-estimate of the
		c           total number of knots of the splines returned, to indicate
		c           the storage space available to the routine. nest >=2*k+2.
		c           in most practical situation nest=m/2 will be sufficient.
		c           always large enough is nest=m+k+1, the number of knots
		c           needed for interpolation (s=0). unchanged on exit.
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
	 */
	void parcur_(int* iopt, int* ipar, int* idim, int* m, double* u, int* mx, double* x, double* w,
			double* ub, double* ue, int* k, double* s, int* nest, int* n, double* t,
			int* nc, double* c, double* fp, double* wrk, int* lwrk, int* iwrk, int* ier);

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


int main(int argc, char** argv) {

	int iopt = 0;				// Smooth spline -- no previous run.
	//int ipar = 0;				// User supplied param values.
	int idim = 2;				// Number of dimensions.
	int k = 3;					// Degree of spline.
	int ib = 2;					// The number of derivative constraints at beginning. Max: (k + 1) / 2
	int ie = 0;					// The number of derivative constraints at end. Max: (k + 1) / 2
	double s = 1;				// Smoothing factor.
	int e = 3;					// Extrapolation

	int n;						// Number of resulting knots.
	//double ue, ub;				// Lower and upper bound of u.
	double fp;					// Weighted sum of squared residuals.
	int ier;					// Error value.

	std::vector<double> u; 		// ? If iopt is zero, need not be supplied.
	std::vector<double> x; 		// Coordinates (multi dimensional).
	std::vector<double> w; 		// Weights.
	std::vector<double> t;		// Knots.
	std::vector<double> c;		// Coefficients.
	std::vector<double> xx;		// Working space.
	std::vector<double> db;		// Constains the constraints on the derivatives from 0->n at beginning.
	std::vector<double> de;		// Constains the constraints on the derivatives from 0->n at end.
	std::vector<double> cp;		// Coefficients with boundary constraints.
	std::vector<double> wrk;	// Work space.
	std::vector<int> iwrk;		// Work space.

	int m = 10;					// Number of points.
	int mx = m * idim;			// Dimension of x array.
	int nest = m + k + 1;		// Estimate of number of knots for storage allotment.
	int nc = nest * idim;		// Dimension of coefficient array.
	int lwrk = m * (k + 1) + nest * (6 + idim + 3 * k);		// Size of work space.
	int nb = std::max(1, ib * idim);	// Dimension of db.
	int ne = std::max(1, ie * idim);	// Dimension of de.
	int np = 2 * (k + 1) * idim;	// Number of coefficients with boundary constraints.

	u.resize(m);
	x.resize(mx);
	xx.resize(mx);
	w.resize(m);
	t.resize(nest);
	c.resize(nc);
	u.resize(m);
	wrk.resize(lwrk);
	iwrk.resize(nest);
	db.resize(nb);
	de.resize(ne);
	cp.resize(np);

	double x_[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
	double yy[] = {1, 2, 3, 4, 5, 4, 3, 2, 1, -1};
	double ww[] = {1, 2, 2, 2, 1, 1, 2, 2, 2, 1};

	db[0] = yy[0];
	db[1] = 0; // 0-th, first and second derivatives.
	db[2] = 0;

	for(int i = 0; i < m; ++i) {
		x[i] = x_[i];
		x[i * idim] = yy[i];
		w[i] = ww[i];
		u[i] = x_[i];
	}

	std::vector<double> y(m);
	int nu1 = 1;
	int nu2 = 2;

	concur_(&iopt, &idim, &m, u.data(), &mx, x.data(), xx.data(), w.data(),
			&ib, db.data(), &nb, &ie, de.data(), &ne,
			&k, &s, &nest, &n, t.data(), &nc, c.data(),
			&np, cp.data(), &fp, wrk.data(), &lwrk, iwrk.data(), &ier);

	splev_(t.data(), &n, c.data(), &k, x.data(), y.data(), &m, &e, &ier);

	db[0] = y[9];

	std::cout << "Y\n";
	for(int i = 0; i < m; ++i)
		std::cout << x[i] << "," << y[i] << " ";
	std::cout << "\n";

	splder_(t.data(), &n, c.data(), &k, &nu1, x.data(), y.data(), &m, &e, wrk.data(), &ier);

	db[1] = y[9];

	std::cout << "D1\n";
	for(int i = 0; i < m; ++i)
		std::cout << x[i] << "," << y[i] << " ";
	std::cout << "\n";

	splder_(t.data(), &n, c.data(), &k, &nu2, x.data(), y.data(), &m, &e, wrk.data(), &ier);

	db[2] = y[9];

	std::cout << "D2\n";
	for(int i = 0; i < m; ++i)
		std::cout << x[i] << "," << y[i] << " ";
	std::cout << "\n";

	concur_(&iopt, &idim, &m, u.data(), &mx, x.data(), xx.data(), w.data(),
			&ib, db.data(), &nb, &ie, de.data(), &ne,
			&k, &s, &nest, &n, t.data(), &nc, c.data(),
			&np, cp.data(), &fp, wrk.data(), &lwrk, iwrk.data(), &ier);

	splev_(t.data(), &n, c.data(), &k, x.data(), y.data(), &m, &e, &ier);

	std::cout << "Y\n";
	for(int i = 0; i < m; ++i)
		std::cout << x[i] << "," << y[i] << " ";
	std::cout << "\n";

	splder_(t.data(), &n, c.data(), &k, &nu1, x.data(), y.data(), &m, &e, wrk.data(), &ier);

	std::cout << "D1\n";
	for(int i = 0; i < m; ++i)
		std::cout << x[i] << "," << y[i] << " ";
	std::cout << "\n";

	splder_(t.data(), &n, c.data(), &k, &nu2, x.data(), y.data(), &m, &e, wrk.data(), &ier);

	std::cout << "D2\n";
	for(int i = 0; i < m; ++i)
		std::cout << x[i] << "," << y[i] << " ";
	std::cout << "\n";

}

