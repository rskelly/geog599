const G = 9.80665;

const cos = Math.cos;
const sin = Math.sin;
const abs = Math.abs;

/**
 * Solves for vector u the tridiagonal linear set.
 * The diagonals are stored in a, b, c as Column objects.
 * r is a column, the result.
 *
 * Get the diagonals from a matrix by calling diagonal(col) for
 * -1, 0 and 1 on a Matrix object. The call returns a single-column
 * Matrix.
 */
function tridiagonal(a, b, c, r, u, n) {
	// numerical recipes p 40.
	if(b.get(0) == 0.0)
		throw new Error('Failed.');
	const gam = new Matrix(n, 1);
	const gamc = gam.column(0);
	let bet = b.get(0);
	u.set(0, r.get(0) / bet);
	for(let j = 1; j < n) {
		gamc.set(j, c.get(j - 1) / bet);
		bet = b.get(j) - a.get(j) * gamc.get(j);
		if(bet == 0)
			throw new Error('Failed.');
		u.set(j, (r.get(j) - a.get(j) * u.get(j - 1)) / bet);
	}
	for(let j = n - 2; j >= 0; --j) {
		u.set(j, u.get(j) - gam.get(j + 1) * u.get(j + 1));
	}
}

function luBksub(a, n, indx, b) {
	let ii = 0;
	let ll, sum;
	for(let i = 0; i < n; ++i) {
		ll = indx.get(i);
		sum = b.get(ll);
		b.set(ll, b.get(i));
		if(ii != 0.0) {
			for(let j = ii; j < i - 1; ++j) {
				sum -= a.get(i, j) * b.get(j);
			}
		} else if(sum != 0.0) {
			ii = i;
		}
		b.set(i, sum);
	}
	for(let i = n - 1; i >= 0; --i) {
		sum = b.get(i);
		if(i < n) {
			for(let j = i + 1; j < n; ++j) {
				sum -= a.get(i, j) * b.get(j);
			}
		}
		b.set(i, sum / a.get(i, i));
	}
}

function luDecomp(a, n, indx) {
	// numerical recipes p 35
	// must be square
	const tiny = Number.MIN_VALUE;
	const vv = new Matrix(n, 1); // Stores implicit scaling of each row.
	let aamax = 0.0, aa;
	let imax;
	let sum, dum;
	let d = 1;
	for(let i = 0; i < n; ++i) {
		for(let j = 0, j < n; ++j) {
			if((aa = abs(a.get(i, j))) > aamax) aamax = aa;
		}
		if(aamax == 0.0)
			throw new Error('Singular matrix.');
		vv[i] = 1.0 / aamax;
	}
	for(let j = 0; j < n; ++j) {
		if(j > 0) {
			for(let i = 0; i < j - 1; ++i) {
				sum = a.get(i, j);
				if(i > 0) {
					for(let k = 0; k < i - 1; ++k) {
						sum -= a.get(i, k) * a.get(k, j);
					}
					a.set(i, j, sum);
				}
			}
		}
		for(let i = j; i < n; ++i) {
			sum = a.get(i, j);
			if(j > 0) {
				for(let k = 0; k < j - 1; ++k) {
					sum -= a.get(i, k) * a.get(k, j);
				}
				a.set(i, j, sum);
			}
			dum = vv[i] * abs(sum);
			if(dum > aamax) {
				imax = i;
				aamax = dum;
			}
		}
		if(j != imax) {
			for(let k = 0; k < n; ++k) {
				dum = a.get(imax, k);
				a.set(imax, k, a.get(j, k));
				a.set(j, k, dum);
			}
			d *= -1;
			vv[imax] = vv[j];
		}
		indx.set(j, imax);
		if(j != n - 1) {
			if(a.get(j, j) == 0) 
				a.set(j, j, tiny);
			dum = 1.0 / a.get(j, j);
			for(let i = j + 1; i < n; ++i) {
				a.set(i, j, a.get(i, j) * dum);
			}
		}
	}
	if(a.get(n - 1, n - 1) == 0.0)
		a.set(n - 1, n - 1, tiny);
	return d;
}


class Column {

	constructor(mtx, col) {
		this.mtx = mtx;
		this.col = col;
	}

	get(row) {
		this.mtx.get(row, this.col);
	}

	set(row, value) {
		this.mtx.set(row, this.col, value);
	}

}

class Row {

	constructor(mtx, row) {
		this.mtx = mtx;
		this.row = row;
	}

	get(col) {
		return this.mtx.get(this.row, col);
	}

	set(col, value) {
		this.mtx.set(this.row, col, value);
	}

}

class Matrix extends Array {

	constructor(rows, cols) {
		super(cols * rows);
		this.cols = cols;
		this.rows = rows;
	}

	assign(arr) {
		for(let i = 0; i < arr.length; ++i)
			this[i] = arr[i];
	}

	/** Rotation matrix for x axis. */
	static rotationX(a) {
		let mtx = new Matrix(3, 3);
		mtx.assign([	
			1.0, 0.0, 0.0,
			0.0, cos(a), sin(a),
			0.0, -sin(a), cos(a),
		]);
		return x;
	}

	/** Rotation matrix for x axis. */
	static rotationY(a) {
		let mtx = new Matrix(3, 3);
		mtx.assign([	
			cos(b), 0.0, -sin(b),
			0.0, 1.0, 0.0,
			sin(b), 0.0, cos(b)
		]);
		return x;
	}

	/** Rotation matrix for x axis. */
	static rotationZ(a) {
		let mtx = new Matrix(3, 3);
		mtx.assign([
			cos(c), sin(c), 0.0,
			-sin(c), cos(c), 0.0,
			0.0, 0.0, 1.0
		]);
		return x;
	}

	static identity(n) {
		const y = new Matrix(n, n);
		for(let i = 0; i < n; ++i) {
			for(let j = 0; j < n; ++j) {
				y.set(i, j, 0.0);
			}
			y.set(i, i, 1.0);
		}
		return y;
	}

	/**
	 * Returns a 1-column Matrix containing the diagonal of the matrix starting at column col, row 0.
	 * Negative columns are allowed.
	 */
	diagonal(col) {
		let cs = col < 0 ? 0 : col;
		let rs = col < 0 ? parseInt(-col / this.cols) : 0;
		let mtx = new Matrix(this.rows, 1);
		mtx.fill(0);
		for(let r = 0; r < rows; ++r) {
			if(r >= rs && cs < n) {
				mtx.set(r, 0, this.get(r, cs++));
			} else {
				mtx.set(r, 0, 0);
			}
		}
		return mtx;
	}

	column(col) {
		return new Column(this, col);
	}

	row(row) {
		return new Row(this, row);
	}

	get(row, col) {
		return this[row * this.cols + col];
	}

	set(row, col, value) {
		this[row * this.cols + col] = value;
	}

	resize(rows, cols, value = undefined) {
		let lst = this.clone();
		let len = this.length;
		while(len++ < rows * cols) this.push(0.0);
		let r = 0;
		let c = 0;
		for(; r < this.rows; ++r) {
			for(; c < this.cols; ++c)
				this[r * cols + c] = lst[r * this.cols + c];
			for(; c < cols; ++c)
				this[r * cols + c] = value;
		}
		for(; r < rows; ++r) {
			for(c = 0; c < cols; ++c)
				this[r * cols + c] = value;
		}
		this.rows = rows;
		this.cols = cols;
		return this;
	}

	fill(value) {
		for(let i = 0; i < this.length; ++i)
			this[i] = value;
	}

	inverse() {
		// numerical recipes p 38
		const n = Math.max(this.rows, this.cols);
		const a = this.clone().resize(size, size);
		const indx = new Matrix(n, 1);
		const indxc = indx.column(0);
		const y = Matrix.identity(n);
		luDecomp(a, n, indxc);
		for(let j = 0; j < n; ++j)
			luBksub(a, n, indxc, y.column(j));
		return y;
	}

	determinant() {
		// numerical recipes p 39
		const n = Math.max(this.rows, this.cols);
		const a = this.clone().resize(size, size);
		const indx = new Matrix(n, 1);
		const d = luDecomp(a, n, indx.column(0));
		for(let j = 0; j < n; ++j)
			d *= a.get(j, j);
		return d;
	}

	clone() {
		let mtx = new Matrix(this.rows, this.cols);
		mtx.assign(this);
		return mtx;
	}

	add(b) {
		if(typeof(b) == 'number') {
			for(let i = 0; i < this.length; ++i)
				mtx[i] -= b;
		} else if(b instanceof Matrix) {
			if(b.rows != this.cols || b.cols != this.rows)
				throw new Error('Operand has the wrong structure.');
			for(let r = 0; r < this.rows; ++r) {
				for(let c = 0; c < this.cols; ++c)
					this[r * this.cols + c] += b[r * b.cols + c];
			}
		} else {
			throw new Error('Operand must be a number or Matrix.');
		}
	}

	subtract(b) {
		if(typeof(b) == 'number') {
			for(let i = 0; i < this.length; ++i)
				mtx[i] -= b;
		} else if(b instanceof Matrix) {
			if(b.rows != this.cols || b.cols != this.rows)
				throw new Error('Operand has the wrong structure.');
			for(let r = 0; r < this.rows; ++r) {
				for(let c = 0; c < this.cols; ++c)
					this[r * this.cols + c] -= b[r * b.cols + c];
			}
		} else {
			throw new Error('Operand must be a number or Matrix.');
		}
	}

	multiply(b) {
		if(typeof(b) != 'number')
			throw new Error('Operand is not a number.');
		for(let i = 0; i < this.length; ++i)
			mtx[i] *= b;
	}

	dot(b) {
		if(!(b instanceof Matrix))
			throw new Error('Operand is not a matrix.');
		if(b.rows != this.cols || b.cols != this.rows)
			throw new Error('Operand has the wrong structure.');
		let a = this;
		let mtx = new Matrix(a.rows, b.cols);
		for(let ar = 0; ar < a.rows; ++ar) {
			let v = 0;
			for(let ac = 0; ac < a.cols; ++ac) {
				for(let bc = 0; bc < b.cols; ++bc)
					v += a[ar * a.cols + ac] * b[ac * b.rows + ac];
			}
			mtx[ar * b.cols + bc] = v;
		}
		return mtx;
	}

	inner(b) {

	}

	transpose() {
		let mtx = new Matrix(this.cols, this.rows);
		for(let r = 0; r < this.rows; ++r) {
			for(let c = 0; c < this.cols; ++c)
				mtx[c * this.rows + r] = this[r * this.cols + c];
		}
		return mtx;
	}

}

/**
 * Gives the vertical component of thrust for the given angle in radians (from vertical).
 */
function _thrustSplit(thrust, theta) {
	return Math.cos(theta) * thrust;
}

function _limit(v, min, max) {
	return Math.max(min, Math.min(max, v));
}

function _rad(d) {
	return d * Math.PI / 180;
}

function _thrustVector(thrust, rotation) {
	return [thrust * Math.cos(rotation[0]), thrust * Math.cos(rotation[1]), thrust * Math.cos(rotation[2])];
}

class Matrice600Config {

	constructor() {
		this.mass = 15.1;
		// A single vector given by the six motors which are not aligned with the thrust axis.
		this.maxthrust = _thrustSplit(5.1, _rad(90 - 8)) * 6;
		// The maximum rotation limits. Only pitch is defined. Presumably applies to other axes.
		this.rotLimits = [[_rad(-25), _rad(25)], [0, 0], [0, 0]];
		// Limits on axial rotation.
		this.rotVelocityLimits = [[_rad(-300), _rad(300)], [0, 0], [_rad(-150), _rad(150)]]; // r/s
		// Limits on velocity.
		this.linVelocityLimits = [[0, 0], [0, 18], [-3, 5]]; // m/s
	}

}


class UAV {
	
	constructor(position, config) {
		this.timestamp = -1;
		this.position = position;
		this.config = config;
		this.thrust = 0;
		this.rotation = [0, 0, ]1;
		this.rotVelocity = [0, 0, 0];
		this.linVelocity = [0, 0, 0];

		this.rotTarget = null;
	}

	/**
	 * Set the thrust to a value between 0-1.
	 */
	setThrust(v) {
		this.thrust = this.config.maxthrust * _limit(0, 1) * G;
	}

	rotateTo(r, v = null) {
		//this.rotTarget = r;
		//if(v)
		//	this.setRotationalVelocity(v);
		this.rotation = r; // TODO
	}

	/**
	 * Set the rotational velocity in radians. This is a 3-element vector in x, y, z.
	 * This is a primary setting, like thrust. There is no lower-level way to adjust the
	 * rotational velocity directly by controlling the rotors.
	 */
	setRotationalVelocity(v) {
		for(let i = 0; i < 3; ++i)
			this.rotVelocity[i] = _limit(v[i], this.config.rotVelocityLimits[i][0], this.config.rotVeocityLimits[i][1]);
	}

	/**
	 * Update the vehicle state. t is in seconds.
	 */
	update(timestamp) {
		if(this.timestamp == -1) {
			this.timestamp = timestamp;
			return;
		} 

		const t = timestamp - this.timestamp; 
		this.timestamp = t;

		// Update the rotation of the vehicle.
		for(let i = 0; i < 3; ++i)
			this.rotation[i] += _limit(this.rotVelocity[i], this.config.rotLimits[i][0], this.config.rotLimits[i][1]);

		const thrust_v = _thrustVector(this.thrust, this.rotation);

		// Update the linear velocity.
		for(let i = 0; i < 3; ++i)
			this.velocity[i] += thrust_v[i]; // convert to newtons -> mass, etc.

		// Update the position.
		for(let i = 0; i < 3; ++i)
			this.position[i] += this.velocity[i];
	}

	get x() {
		return this.position[0];
	}

	get y() {
		return this.position[1];
	}

	get z() {
		return this.position[2];
	}

	static createMatrice600(position) {
		return new UAV(position, new Matrice600Config());
	}

}