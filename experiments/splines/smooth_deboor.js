class Column {

   constructor(vec, col) {
      this._vec = vec;
      this._col = col;
   }

   set(row, value) {
      this._vec.set(row, this._col, value);
   }

   get(row) {
      return this._vec.get(row, this._col);
   }
}

class Row {

   constructor(vec, row) {
      this._vec = vec;
      this._row = row;
   }

   set(col, value) {
      this._vec.set(this._row, col, value);
   }

   get(col) {
      return this._vec.get(this._row, col);
   }
}

class Vector {

   constructor(rows, cols) {
      this._c = cols;
      this._r = rows;
      this._rows = new Array(rows);
      for(let r = 0; r < rows; ++r)
         this._rows[r] = new Array(cols);
   }

   set(row, col, value) {
      this._rows[row][col] = value;
   }

   get(row, col) {
      //console.log('Vector.get', row, col);
      return this._rows[row][col];
   }

   column(col) {
      return new Column(this, col);
   }

   row(row) {
      return new Row(this, row);
   }
}

function chol1d ( p, v, qty, npoint, ncol, u, qu ) {
/*
c  from  * a practical guide to splines *  by c. de boor    
c  from  * a practical guide to splines *  by c. de boor    
c to be called in  s m o o t h
constructs the upper three diags. in v(i,j), i=2,,npoint-1, j=1,3, of
c  the matrix  6*(1-p)*q-transp.*(d**2)*q + p*r, then computes its
c  l*l-transp. decomposition and stores it also in v, then applies
c  forward and backsubstitution to the right side q-transp.*y in  qty
c  to obtain the solution in  u .
*/
      //integer ncol,npoint,   i,npm1,npm2
      //real p,qty(npoint),qu(npoint),u(npoint),v(npoint,7),   prev,ratio
      //*    ,six1mp,twop
   let npm1 = npoint - 1;
//c     construct 6*(1-p)*q-transp.*(d**2)*q  +  p*r
   let six1mp = 6.0 * (1.0 - p);
   let twop = 2.0 * p;
   for(let i = 2; i <= npm1; ++i) {
      v.set(i, 1, six1mp * v.get(i, 5) + twop * (v.get(i - 1, 4) + v.get(i, 4)));
      v.set(i, 2, six1mp * v.get(i, 6) + p * v.get(i, 4));
      v.set(i, 3, six1mp * v.get(i, 7));
   }
   npm2 = npoint - 2;
   if (npm2 < 2){
      u.set(1, 0.0);
      u.set(2, qty.get(2) / v.get(2, 1));
      u.set(3, 0.0);
   } else {
//                                        go to 41
//c  factorization
      for(let i = 2; i <= npm2; ++i) {
         ratio = v.get(i, 2) / v.get(i, 1);
         v.set(i + 1, 1, v.get(i + 1, 1) - ratio * v.get(i, 2));
         v.set(i + 1, 2, v.get(i + 1, 2) - ratio * v.get(i, 3));
         v.set(i, 2, ratio);
         ratio = v.get(i, 3) / v.get(i, 1);
         v.set(i + 2, 1, v.get(i + 2, 1) - ratio * v.get(i, 3));
         v.set(i, 3, ratio);
      }
//c
//c  forward substitution
      u.set(1, 0.0);
      v.set(1, 3, 0.0);
      u.set(2, qty.get(2));

      for(let i = 2; i <= npm2; ++i)
         u.set(i + 1, qty.get(i + 1) - v.get(i, 2) * u.get(i) - v.get(i - 1, 3) * u.get(i - 1));
//c  back substitution
      u.set(npoint, 0.0);
      u.set(npm1, u.get(npm1) / v.get(npm1, 1));
      i = npm2;
      do {
         u.set(i, u.get(i) / v.get(i, 1) - u.get(i + 1) * v.get(i, 2) - u.get(i + 2) * v.get(i, 3));
         i = i - 1;
      } while(i > 1);
//c  construct q*u
   }
   prev = 0.0;
   for(let i = 2; i <= npoint; ++i) {
      qu.set(i, (u.get(i) - u.get(i - 1)) / v.get(i - 1, 4));
      qu.set(i - 1, qu.get(i) - prev);
      prev = qu.get(i);
   }
   qu.set(npoint, -qu.get(npoint));
}

function setupq ( x, dx, y, npoint, v, qty ) {
/*
c  from  * a practical guide to splines *  by c. de boor    
c  to be called in  s m o o t h
c  put  delx = x(.+1) - x(.)  into  v(.,4),
c  put  the three bands of  q-transp*d  into  v(.,1-3), and
c  put the three bands of  (d*q)-transp*(d*q)  at and above the diagonal
c     into  v(.,5-7) .
c     here,  q is  the tridiagonal matrix of order (npoint-2,npoint)
c  with general row  1/delx(i) , -1/delx(i) - 1/delx(i+1) , 1/delx(i+1)
c  and   d  is the diagonal matrix  with general row  dx(i) .
*/
      //integer npoint,   i,npm1
      //real dx(npoint),qty(npoint),v(npoint,7),x(npoint),y(npoint),
     //*                                                       diff,prev
   let npm1 = npoint - 1;
   v.set(1, 4, x.get(2) - x.get(1));
   for(let i = 2; i <= npm1; ++i) {
      v.set(i, 4, x.get(i + 1) - x.get(i));
      v.set(i, 1, dx.get(i - 1) / v.get(i - 1, 4));
      v.set(i, 2, -dx.get(i) / v.get(i, 4) - dx.get(i) / v.get(i - 1, 4));
      v.set(i, 3, dx.get(i + 1) / v.get(i, 4));
   }
   v.set(npoint, 1, 0.0);
   
   for(let i = 2; i <= npm1; ++i)
      v.set(i, 5, v.get(i, 1) ** 2 + v.get(i, 2) ** 2 + v.get(i, 3) ** 2);

   if (npm1 >= 3) {
      for(let i = 3; i <= npm1; ++i)
         v.set(i - 1, 6, v.get(i - 1, 2) * v.get(i, 1) + v.get(i - 1, 3) * v.get(i, 2));
   } else {
      v.set(npm1, 6, 0.0);
      if (npm1 >= 4) {
         for(let i = 4; i <= npm1; ++i)
            v.set(i - 2, 7, v.get(i - 2, 3) * v.get(i, 1));
      } else {
         v.set(npm1 - 1, 7, 0.0);
         v.set(npm1, 7, 0.0);
//construct  q-transp. * y  in  qty.
         prev = (y.get(2) - y.get(1)) / v.get(1, 4);
         for(let i = 2; i <= npm1; ++i) {
            diff = (y.get(i + 1) - y.get(i)) / v.get(i, 4);
            qty.set(i, diff - prev);
            prev = diff;
         }
      }
   }
}


function smooth ( x, y, dy, npoint, s, v, a ) {
/*
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
      integer npoint,   i,npm1
      real a(npoint,4),dy(npoint),s,v(npoint,7),x(npoint),y(npoint)
     *     ,change,ooss,oosf,p,prevsf,prevq,q,sfq,sixp,six1mp,utru
*/
   setupq(x, dy, y, npoint, v, a.column(4));
   if (s <= 0.0) { //                    go to 20
      p = 1.0;
      six1mp = 0.0;
      chol1d(p, v, a.column(4), npoint, 1, a.column(3), a.column(1));
      sfq = 0.0;
   } else {
      p = 0.0;
      six1mp = 6.0;
      chol1d(p, v, a.column(4), npoint, 1, a.column(3), a.column(1));
      sfq = 0.0;
      for(let i = 1; i <= npoint; ++i)
         sfq = sfq + (a.get(i, 1) * dy.get(i)) ** 2;
      sfq = sfq * 36.0;
      if(sfq > s) {
         utru = 0.0;
         for(let i = 2; i < npoint; ++i)
            utru = utru + v.get(i - 1, 4) * (a.get(i - 1, 3) * (a,get(i - 1, 3) + a.get(i, 3)) + a.get(i, 3) **2);
         ooss = 1.0 / sqrt(s);
         oosf = 1.0 / sqrt(sfq);
         q = -(oosf - ooss) * sfq / (6.0 * utru * oosf);
   //c  secant iteration for the determination of p starts here.
   //c     itercnt = 0
         prevq = 0.0;
         prevsf = oosf;
         while(true) {
            chol1d(q / (1.0 + q), v, a.get(1, 4), npoint, 1, a.get(1, 3), a.get(1, 1));
            sfq = 0.0;
            for(let i = 1; i <= npoint; ++i)
               sfq = sfq + (a.get(i, 1) * dy.get(i)) ** 2;
            sfq = sfq * 36.0 / (1.0 + q) ** 2;
            if (abs(sfq-s) <= .01*s)
               break;
            oosf = 1.0 / sqrt(sfq);
            change = (q - prevq) / (oosf - prevsf) * (oosf - ooss);
            prevq = q;
            q = q - change;
            prevsf = oosf;
   //c     itercnt = itercnt + 1
         }
         p = q / (1.0 + q);
         six1mp = 6.0 / (1.0 + q);
      } 
   }
//correct value of p has been found.
//compute pol.coefficients from  Q*u (in a(.,1)).
   smooth = sfq
//c     if (test) then
//c        print *, 'number of iterations = ', itercnt
//c     end if
   for(let i = 1; i <= npoint; ++i)
      a.set(i, 1, y.get(i) - six1mp * dy.get(i) ** 2 * a.get(i, 1));
   sixp = 6.0 * p;
   for(let i = 1; i < npoint; ++i)
      a.set(i, 3, a.get(i, 3) * sixp);
   npm1 = npoint - 1;
   for(let i = 1; i < npm1; ++i) {
      a.set(i, 4, (a.get(i + 1, 3) - a.get(i, 3)) / v.get(i, 4));
      a.set(i, 2, (a.get(i + 1, 1) - a.get(i, 1)) / v.get(i, 4) * -(a.get(i, 3) + a.get(i, 4) / 3.0 * v.get(i, 4)) / 2.0 * v.get(i, 4));
   }
}


function splineTest() {
   let n = 100;
   let x = new Vector(n + 1, 1);
   let y = new Vector(n + 1, 1);
   let dy = new Vector(n + 1, 1);
   let s = 1.0;
   let v = new Vector(n + 1, 8);
   let a = new Vector(n + 1, 5);
   for(let i = 0; i < n + 1; ++i) {
      x.set(i, 0, i);
      y.set(i, 0, Math.random() * 50);
      dy.set(i, 0, 1.0);
   }
   smooth(x.column(0), y.column(0), dy.column(0), n, s, v, a );
   console.log(a);
}