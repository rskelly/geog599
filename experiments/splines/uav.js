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
		this.rotation = [0, 0, 0];
		this.rotVelocity = [0, 0, 0];
		this.linVelocity = [0, 0, 0];
	}

	/**
	 * Set the thrust to a value between 0-1.
	 */
	setThrust(v) {
		this.thrust = this.config.maxthrust * _limit(0, 1);
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