const _p = Math.pow;
const _s = Math.sqrt;
const _min = Math.min;
const _max = Math.max;

function _p3(t) {
  return t*t*t;
}

function _p2(t) {
  return t*t;
}


function crTJ(ti, pi, pj, alpha) {
  let a = _p(pj.y - pi.y, 2.0) + _p(pj.z - pi.z, 2.0);
  let b = _s(a);
  let c = _p(b, alpha);
  return ti + c;
}

function catmullRomSplineT(t, p0, p1, p2, p3, alpha = 0.5, npoints = 10) {

  const t0 = 0.0;
  const t1 = crTJ(t0, p0, p1, alpha);
  const t2 = crTJ(t1, p1, p2, alpha);
  const t3 = crTJ(t2, p2, p3, alpha);

  let a1 = p0.times((t1 - t) / (t1 - t0)).plus(p1.times((t - t0) / (t1 - t0)));
  let a2 = p1.times((t2 - t) / (t2 - t1)).plus(p2.times((t - t1) / (t2 - t1)));
  let a3 = p2.times((t3 - t) / (t3 - t2)).plus(p3.times((t - t2) / (t3 - t2)));
  let b1 = a1.times((t2 - t) / (t2 - t0)).plus(a2.times((t - t0) / (t2 - t0)));
  let b2 = a2.times((t3 - t) / (t3 - t1)).plus(a3.times((t - t1) / (t3 - t1)));
  let c  = b1.times((t2 - t) / (t2 - t1)).plus(b2.times((t - t1) / (t2 - t1)));

  return c;
}

function catmullRomSpline(p0, p1, p2, p3, alpha = 0.5, npoints = 10) {

  const t0 = 0.0;
  const t1 = crTJ(t0, p0, p1, alpha);
  const t2 = crTJ(t1, p1, p2, alpha);
  const t3 = crTJ(t2, p2, p3, alpha);

  let out = []

  for(let t = t1; t < t2; t += (t2 - t1) / npoints) {

    let a1 = p0.times((t1 - t) / (t1 - t0)).plus(p1.times((t - t0) / (t1 - t0)));
    let a2 = p1.times((t2 - t) / (t2 - t1)).plus(p2.times((t - t1) / (t2 - t1)));
    let a3 = p2.times((t3 - t) / (t3 - t2)).plus(p3.times((t - t2) / (t3 - t2)));
    let b1 = a1.times((t2 - t) / (t2 - t0)).plus(a2.times((t - t0) / (t2 - t0)));
    let b2 = a2.times((t3 - t) / (t3 - t1)).plus(a3.times((t - t1) / (t3 - t1)));
    let c  = b1.times((t2 - t) / (t2 - t1)).plus(b2.times((t - t1) / (t2 - t1)));

    out.push(c);

  }      

  return out;
}

function getCRSplines(points, alpha = 0.95, npoints = 10) {
  let out = [];
  for(let i = 0; i < points.length - 3; ++i)
    Array.prototype.push.apply(out, catmullRomSpline(points[i], points[i + 1], points[i + 2], points[i + 3], alpha, npoints));
  return out;
}

// Tests which side of the line, described by p0 and p1, p is on.
function _cross(p0, p1, p) {
  return (p0.y - p.y) * (p1.z - p.z) - (p0.z - p.z) * (p1.y - p.y);
}

// Sort the points on x first, then y.
function _sort(p0, p1) {
  return p0.y == p1.y ? p0.z > p1.z : p0.y > p1.y;
}

// Return the squared distance between the points, but only in x.
function _lengthY(p0, p1) {
  return _p(p0.y - p1.y, 2);
}

// Finds the maximum z within winsize Cartesian distance of the point at idx.
function _inwindow(pts, idx, winsize) {
  let z = pts[idx].z;
  let out = idx;
  for(let i = idx + 1; i < pts.length; ++i) {
    if(_length(pts[idx], pts[i]) <= winsize) {
      if(pts[i].z > z)
        out = i;
    } else {
      break;
    }
  }
  for(let i = idx - 1; i >= 0; --i) {
    if(_length(pts[idx], pts[i]) <= winsize) {
      if(pts[i].z > z)
        out = i;
    } else {
      break;
    }
  }
  return pts[out];
}

function getHull(pts, alpha) {
  alpha *= alpha;
  let hull = [pts[0], pts[1]];
  for(let i = 2; i < pts.length; ++i) {
    // The _length call limits the range of the search for a convex point; causes an alpha-like surface.
    let c = _cross(hull[hull.length - 2], hull[hull.length - 1], pts[i]);
    let l = _lengthY(hull[hull.length - 2], pts[i]);
    while(hull.length >= 2 && (c = _cross(hull[hull.length - 2], hull[hull.length - 1], pts[i])) >= 0 && (alpha <= 0 || (l = _lengthY(hull[hull.length - 2], pts[i])) <= alpha))
      hull.pop();
    hull.push(pts[i]);
  }
  /*
  for(let i = 1; i < hull.length - 3; ++i) {
    if(_length(hull[i], hull[i + 1]) < alpha) {
      if(_length(hull[i - 1], hull[i]) < _length(hull[i + 1], hull[i + 2])) {
        hull[i].z = _max(hull[i].z, hull[i + 1].z);
        hull.splice(i, 1);
        --i;
      } else {
        hull[i + 1].z = _max(hull[i].z, hull[i + 1].z);
        hull.splice(i + 1, 1);
      }
    }
  }
  */
  return hull;
}

function getNormal2(pt0, pt1, length = 10) {
  let dx = pt1.x - pt0.x;
  let dz = pt1.z - pt0.z;
  let h = _s(_p2(dx) + _p2(dz));
  let p = length / h;
  let pt2 = new Point(pt0.x + dx / 2.0, 0, pt0.z + dz / 2.0);
  if(dx > 0) p = -p; // If the normal slopes down, reverse it. dx controls the z because the slope is reversed in the normal.
  let pt3 = new Point(pt2.x + p * dz, 0, pt2.z + p * -dx);
  return [pt2, pt3];
}

function getNormal3(pt0, pt1, pt2, length = 10) {
  // These two angles are on the underside, relative to vertical.
  let a0 = Math.atan2(pt0.z - pt1.z, pt0.x - pt1.x);  
  let a1 = Math.atan2(pt2.z - pt1.z, pt2.x - pt1.x);
  let a = a0 - ((a0 - a1) / 2.0);
  while(a < 0.0) a += Math.PI * 2.0;
  while(a > Math.PI * 2.0) a -= Math.PI * 2.0;
  if(a > Math.PI) a -= Math.PI;
  let pt = new Point(pt1.x + Math.cos(a) * length, 0, pt1.z + Math.sin(a) * length);
  return [pt1, pt];
}

function getSlope(pt0, pt1) {
  return (pt1.z - pt0.z) / (pt1.x - pt0.x);
}

function filterHull(hull) {
  // Remove clusters from the hull. Take the highest elevation in the cluster and replace the remaining
  // null nodes' z value.
  let hull0 = [hull[0].clone()];
  for(let i = 1; i < hull.length; ++i) {
    let m = -1 / getSlope(hull[i - 1], hull[i])
    console.log(m, Math.abs(m));
    if(Math.abs(m) > 1)
      hull0.push(hull[i - 1].z > hull[i].z ? hull[i - 1] : hull[i]);
  }
  if(!hull0[0].equals(hull[hull.length - 1]))
    hull0.push(hull[hull.length - 1]);
  return hull0;
}

