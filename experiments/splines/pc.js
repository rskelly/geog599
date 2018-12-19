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

class PointCloud {

	constructor(points = null) {
		this.reset();
		if(points)
			points.forEach(pt => { this.addPoint(pt.clone()); });
	}

	reset() {
		this.points = [];
		this.bounds = new Bounds();
	}

	addPoint(pt) {
        this.points.push(pt);
        this.bounds.extend(pt);		
	}

	sortY() {
		this.points.sort(_sortY);
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
    	if(!pcbounds) pcbounds = this.bounds;
    	let b = new Bounds();
    	for(let i = 0; i < this.points.length; ++i) {
    		let pt = this.points[i];
    		pt.x = pcbounds.width == 0 ? 0 : ((pt.x - pcbounds.min.x) / pcbounds.width) * scrbounds.width + scrbounds.min.x;
    		pt.y = pcbounds.length == 0 ? 0 : ((pt.y - pcbounds.min.y) / pcbounds.length) * scrbounds.length + scrbounds.min.y;
    		pt.z = pcbounds.height == 0 ? 0 : ((pt.z - pcbounds.min.z) / pcbounds.height) * scrbounds.height + scrbounds.min.z;
    		b.extend(pt);
    	}
    	this.sortY();
    	this.bounds = b;
    	return this;
    }

    sliceY(y0, y1) {
    	let pc = new PointCloud();
    	this.points.forEach(pt => {
    		if(pt.y >= y0 && pt.y < y1)
    			pc.addPoint(pt.clone());
    	});
    	return pc;
    }

    sliceYRay(y, z, angle, tolerance = Math.PI / 180) {
    	let pc = new PointCloud();
    	this.points.forEach(pt => {
    		let t = Math.atan2(pt.z - z, pt.y - y);
    		if(Math.abs(t - angle) <= tolerance)
    			pc.addPoint(pt.clone());
    	});
    	return pc;
    }

}