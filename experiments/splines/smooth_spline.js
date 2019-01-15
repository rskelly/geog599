// http://www.physics.muni.cz/~jancely/NM/Texty/Numerika/CubicSmoothingSpline.pdf

function sqr(a) {
	return a*a;
}

function quincunx(n /* integer */, u, v, w, q /* nvectors */) {
	
	let uu = u.slice();
	uu[0] = 0;
	uu.unshift(0);
	let vv = v.slice();
	vv.unshift(0);
	let ww = w.slice();
	ww.unshift(0);
	let qq = q.slice();
	qq.unshift(0);

	for(let j = 2; j < n + 1; ++j) {
		uu[j] = uu[j] - uu[j - 2] * sqr(ww[j - 2]) - uu[j - 1] * sqr(vv[j - 1]);
		vv[j] = (vv[j] - uu[j - 1] * vv[j - 1] * ww[j - 1]) / uu[j];
		ww[j] = ww[j] / uu[j];
	}

	for(let j = 2; j < n + 1; ++j)
		qq[j] = qq[j] - vv[j - 1] * qq[j - 1] - ww[j - 1] * qq[j - 1];

	for(let j = 2; j < n + 1; ++j)
		qq[j] = qq[j] / uu[j];

	qq.push(0);
	qq.push(0);
	vv.push(0);
	vv.push(0);
	ww.push(0);
	ww.push(0);

	for(let j = n; j >= 2; --j)
		qq[j] = qq[j] - vv[j] * qq[j + 1] - ww[j] * qq[j + 2];

	for(let j = 0; j < n; ++j) {
		u[j] = uu[j + 1];
		v[j] = vv[j + 1];
		w[j] = ww[j + 1];
		q[j] = qq[j + 1];
	}

}

function smoothingSpline(spline /* SplineVec */, sigma /* vectors */, lambda /* real */, n /* int */) {

	let h = [0], r = [0], f = [0], p = [0], q = [0], u = [0], v = [0], w = [0]; /* vectors */
	let mu; /* real */

	mu = 2 * (1 - lambda) / (3 * lambda);
	h[0] = spline[1].x - spline[0].x;
	r[0] = 3 / h[0];

	for(let i = 1; i < n; ++i) {
		h[i] = spline[i + 1].x - spline[i].x;
		r[i] = 3 / h[i];
		f[i] = -(r[i - 1] + r[i]);
		p[i] = 2 * (spline[i + 1].x - spline[i - 1].x);
		q[i] = 3 * (spline[i + 1].y - spline[i].y) / h[i] - 3 * (spline[i].y - spline[i - 1].y) / h[i - 1];
	}

	for(let i = 1; i < n; ++i) {
		u[i] = sqr(r[i - 1]) * sigma[i - 1] + sqr(f [i]) * sigma[i] + sqr(r[i]) * sigma[i + 1];
		u[i] = mu * u[i] + p[i];
		v[i] = f[i] * r[i] * sigma[i] + r[i] * f [i + 1] * sigma[i + 1];
		v[i] = mu * v[i] + h[i];
		w[i] = mu * r[i] * r[i + 1] * sigma[i + 1];
	}

	quincunx(n, u, v, w, q);

	spline[0].d = spline[0].y - mu * r[0] * q[1] * sigma[0];
	spline[1].d = spline[1].y - mu * (f [1] * q[1] + r[1] * q[2]) * sigma[0];
	spline[0].a = q[1] / (3 * h[0]);
	spline[0].b = 0;
	spline[0].c = (spline[1].d - spline[0].d) / h[0] - q[1] * h[0] / 3;
	r[0] = 0;

	for(let j = 1; j < n; ++j) {
		spline[j].a = (q[j + 1] - q[j]) / (3 * h[j]);
		spline[j].b = q[j];
		spline[j].c = (q[j] + q[j - 1]) * h[j - 1] + spline[j - 1].c;
		spline[j].d = r[j - 1] * q[j - 1] + f [j] * q[j] + r[j] * q[j + 1];
		spline[j].d = y[j] - mu * spline[j].d * sigma[j];
	}
}

function splineTest() {
	let spline = [];
	let sigma = [];
	let lambda = 0.5;
	let n = 100;
	for(let i = 0; i < n + 1; ++i) {
		spline.push({
			x: i, y: Math.random() * 50,
			a: 0, b: 0, c: 0, d: 0
		});
		sigma.push(0.5);
	}
	smoothingSpline(spline, sigma, lambda, n);
}