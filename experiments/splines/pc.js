const DEFAULT_BOUNDS = [Number.MAX_VALUE, -Number.MAX_VALUE, Number.MAX_VALUE, -Number.MAX_VALUE, Number.MAX_VALUE, -Number.MAX_VALUE];

function _len(p1, p2) {
	return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}

function _sortY(p1, p2) {
	return p1.y == p2.y ? p1.z > p2.z : p1.y > p2.y;
}

class Point {
  constructor(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
  times(s) {
    return new Point(this.x * s, this.y * s, this.z * s);
  }
  plus(p) {
    return new Point(this.x + p.x, this.y + p.y, this.z + p.z);
  }
  clone() {
    return new Point(this.x, this.y, this.z);
  }
  equals(p) {
    return this.x == p.x && this.y == p.y && this.z == p.z;
  }

  /**
   * Scales list of points to fit in the given bounds.
   */
  static scale(pts, scrbounds, pcbounds = null) {
    if(!pcbounds) {
      pcbounds = new Bounds();
      pts.forEach(pt => pcbounds.extend(pt));
    }
    let b = new Bounds();
    for(let i = 0; i < pts.length; ++i) {
      let pt = pts[i];
      pt.x = pcbounds.width == 0 ? 0 : ((pt.x - pcbounds.min.x) / pcbounds.width) * scrbounds.width + scrbounds.min.x;
      pt.y = pcbounds.length == 0 ? 0 : ((pt.y - pcbounds.min.y) / pcbounds.length) * scrbounds.length + scrbounds.min.y;
      pt.z = pcbounds.height == 0 ? 0 : ((pt.z - pcbounds.min.z) / pcbounds.height) * scrbounds.height + scrbounds.min.z;
      b.extend(pt);
    }
    return [pts, b];
  }
}

class Bounds {

	constructor(xmin = Number.MAX_VALUE, xmax = -Number.MAX_VALUE, ymin = Number.MAX_VALUE, ymax = -Number.MAX_VALUE, 
		zmin = Number.MAX_VALUE, zmax = -Number.MAX_VALUE) {
		this.xmin = xmin;
		this.xmax = xmax;
		this.ymin = ymin;
		this.ymax = ymax;
		this.zmin = zmin;
		this.zmax = zmax;
	}

	collapse() {
		this.xmin = Number.MAX_VALUE;
		this.ymin = Number.MAX_VALUE;
		this.zmin = Number.MAX_VALUE;
		this.xmax = -Number.MAX_VALUE;
		this.ymax = -Number.MAX_VALUE;
		this.zmax = -Number.MAX_VALUE;
	}

	extend(pt) {
		if(pt.x < this.xmin) this.xmin = pt.x;
		if(pt.x > this.xmax) this.xmax = pt.x;
		if(pt.y < this.ymin) this.ymin = pt.y;
		if(pt.y > this.ymax) this.ymax = pt.y;
		if(pt.z < this.zmin) this.zmin = pt.z;
		if(pt.z > this.zmax) this.zmax = pt.z;
	}

	get width() {
		return this.xmax - this.xmin;
	}

	get length() {
		return this.ymax - this.ymin;
	}

	get height() {
		return this.zmax - this.zmin;
	}

	get min() {
		return new Point(this.xmin, this.ymin, this.zmin);
	}

	get max() {
		return new Point(this.xmax, this.ymax, this.zmax);
	}

	intersects(b) {

	}

	intersection(b) {

	}

	toString() {
		return [this.xmin, this.xmax, this.ymin, this.ymax, this.zmin, this.zmax].join(',');
	}

	clone() {
		return new Bounds(this.xmin, this.xmax, this.ymin, this.ymax, this.zmin, this.zmax);
	}

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

class PointCloud {

	constructor(points = null) {
		this.reset();
    this._set = new Set();
		if(points) {
			points.forEach(pt => { this.addPoint(pt.clone()); });
      this.sortY();
    }
	}

	reset() {
		this.points = [];
		this.bounds = new Bounds();
	}

	addPoint(pt) {
    if(!this._set.has(pt)) {
      this.points.push(pt);
      this.bounds.extend(pt);		
    }
	}

	sortY() {
		this.points.sort(_sortY);
	}

  filterY(ymin = -Number.MAX_VALUE, ymax = Number.MAX_VALUE) {
    this.points = this.points.filter(pt => (pt.y >= ymin && pt.y <= ymax));
    this.bounds.collapse();
    this.points.forEach(pt => this.bounds.extend(pt));
  }

	/**
	 * Load the given text file containing points as x, y, z.
	 * The callback is a status indicator. The first argument is true
	 * if the load is complete, false otherwise. The second arg is a message.
	 */
	load(file, callback = null) {
      if(callback)
      	callback(false, 'Loading...');
      this.reset();
      let reader = new XMLHttpRequest();
      reader.overrideMimeType('text/plain');
      reader.open('GET', file);
      reader.addEventListener('load', evt => {
        if(reader.readyState == 4) {
          let rows = reader.responseText.split(/\n/);
          rows.forEach(row => {
            row = row.split(',').slice(0, 3).map(parseFloat);
            let pt = new Point(row[0], row[1], row[2]);
            if(isNaN(pt.x)) return;
            this.addPoint(pt);
          });
          this.sortY();
          if(callback)
          	callback(true, '');
        }
      });
      reader.send();
    }

    /**
     * Transforms the point cloud so that the x coordinates are zero, the y coordinates
     * represent the horizontal distance and the z stay the same.
     */
    transform() {
    	let b = new Bounds();
    	let maxlen = _len(this.bounds.min, this.bounds.max);
    	for(let i = 0; i < this.points.length; ++i) {
    		let pt = this.points[i];
    		pt.y = _len(pt, this.bounds.min);
    		pt.x = 0.0;
    		b.extend(pt);
    	}
        this.sortY();
        this.bounds = b;
        return this;
    }

    /**
     * Scales point cloud to fit in the given bounds.
     */
    scale(scrbounds, pcbounds = null) {
      let [pts, bounds] = Point.scale(this.points, scrbounds, pcbounds || this.bounds);
      this.points = pts;
      this.bounds = bounds;
    	this.sortY();
    	return this;
    }

    /**
     * Return a slice of the point cloud betweem y0 and y1.
     */
    sliceY(y0, y1) {
    	let pc = new PointCloud();
    	this.points.forEach(pt => {
    		if(pt.y >= y0 && pt.y < y1)
    			pc.addPoint(pt.clone());
    	});
    	return pc;
    }

    /**
     * Return a slice of the point cloud along the ray of given angle.
     * The tolerance is an angle on either side of the ray within which
     * points are captured. The ray begins at y,z.
     */
    sliceYRay(y, z, angle, tolerance = Math.PI / 180) {
    	let pc = new PointCloud();
    	this.points.forEach(pt => {
    		let t = Math.atan2(pt.z - z, pt.y - y);
    		if(Math.abs(t - angle) <= tolerance)
    			pc.addPoint(pt.clone());
    	});
    	return pc;
    }

    /**
     * Return a list of points surrounding the given y coordinate.
     * The list will include up to num points on ether side of y. 
     * If one of the two inner points has a y equal to the given y,
     * it is placed to the left.
     */
    sliceSeg(y, num) {
      let n = parseInt(num / 2);
      let pc = new PointCloud();
      for(let i = n; i < this.points.length - num; ++i) {
        if(y >= this.points[i].y && y < this.points[i + 1].y) {
          for(let j = i - n + 1; j < i + n + 1; ++j)
            pc.addPoint(this.points[j].clone());
          break;
        }
      }
      return pc;
    }

    getHull(alpha) {
      if(this.points.length < 3)
        return new PointCloud([]);
      alpha *= alpha;
      const pts = this.points;
      pts.sort(_sortY);
      let hull = [pts[0], pts[1]];
      for(let i = 2; i < pts.length; ++i) {
        // The _length call limits the range of the search for a convex point; causes an alpha-like surface.
        let c, l;
        while(hull.length >= 2 && (c = _cross(hull[hull.length - 2], hull[hull.length - 1], pts[i])) >= 0 && (alpha <= 0 || (l = _lengthY(hull[hull.length - 2], pts[i])) <= alpha))
          hull.pop();
        hull.push(pts[i]);
      }
      let len;
      do {
        len = hull.length;
        for(let i = 1; i < hull.length - 1; ++i) {
          for(let j = i + 1; j < hull.length; ++j) {
            console.log(i, j);
            if(_lengthY(hull[i], hull[j]) < alpha) {
              if(hull[i].z > hull[j].z) {
                hull.splice(j, 1);
                --j;
              } else {
                hull.splice(i, 1);
                --i;
              }
            }
          }
        }
      } while(len != hull.length);
      return new PointCloud(hull);
    }

    clone() {
      let pc = new PointCloud();
      this.points.forEach(pt => pc.addPoint(pt.clone()));
      return pc;
    }
}