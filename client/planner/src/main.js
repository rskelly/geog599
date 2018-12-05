const L = require('leaflet');

const BASEMAP_URL = 'https://api.mapbox.com/styles/v1/rskelly/cin2fy4x4001latnlqi4yu13e/wmts?access_token=pk.eyJ1IjoicnNrZWxseSIsImEiOiJjaWkzeHBqbWgwMTV2dHJtMGpoandtbXZ1In0.NfvC9sMFzimyq1zeq-rHkA';
const MB_TOKEN = 'pk.eyJ1IjoicnNrZWxseSIsImEiOiJjaWkzeHBqbWgwMTV2dHJtMGpoandtbXZ1In0.NfvC9sMFzimyq1zeq-rHkA';

const __bound = new Map();

/**
 * Select a child of the element by selector.
 * sel - The selector.
 * el - The element or null.
 */
function _qs(sel, el) {
	return (el || document).querySelector(sel);
}

/**
 * Select the children of the element by selector.
 * sel - The selector.
 * el - The element or null.
 */
function _qsa(sel, el) {
	return (el || document).querySelectorAll(sel);
}

/**
 * Listen for events on the element using the callback.
 * el - The element.
 * evt - The name of the event.
 * callback - The function to call.
 */
function _listen(el, evt, callback) {
	el.addEventListener(evt, callback);
	return el;
}

/**
 * Bind a function to the object.
 * owner - The object to bind to.
 * fn - The function.
 */
function _bind(owner, fn) {
	if(__bound.has(fn)) {
		if(__bound.get(fn).has(owner)) {
			return __bound.get(fn).get(owner);
		} else {
			const f = fn.bind(owner);
			__bound.get(fn).set(owner, f);
			return f;
		}
	} else {
		const f = fn.bind(owner);
		const m = new Map();
		m.set(owner, f);
		__bound.set(fn, m);
		return f;
	}
}

/**
 * Unbind a function.
 * owner - The object bound to.
 * fn - The function.
 */
function _unbind(owner, fn) {
	if(__bound.has(fn)) {
		if(__bound.get(fn).has(owner))
			__bound.get(fn).delete(owner);
		if(__bound.get(fn).size == 0)
			__bound.delete(fn);
	}
}

function _clear(el) {
	while(el.firstChild)
		el.removeChild(el.firstChild);
}

function _node(type) {
	return document.createElement(type);
}

function _text(el, text) {
	el.appendChild(document.createTextNode(text));
	return el;
}

class EvtDisp {

	constructor() {
		this.__listeners = new Map();
	}

	on(name, callback) {
		if(!this.__listeners.has(name))
			this.__listeners.set(name, new Set());
		this.__listeners.get(name).add(callback);
	}

	off(name, callback) {
		if(this.__listeners.has(name))
			this.__listeners.get(name).delete(callback);
	}

	dispatch(name, evt = {}) {
		evt.target = this;
		if(this.__listeners.has(name)) {
			for(let callback of this.__listeners.get(name))
				callback(evt);
		}
	}

}

class Datum extends EvtDisp {

	constructor() {
		super();
	}

	select() {
		this.dispatch('select');
	}

	erase() {
		this.dispatch('erase');
	}

	update(fields = []) {
		this.dispatch('update', {fields: fields});
	}

}


class SimplePolygon extends Datum {

	constructor() {
		super();
		this.points = [];
		this._points = new Set();
	}

	addPoint(x, y) {
		this.points.push([x, y]);
		this.update(['points']);
	}

	removePoint(x, y) {
		let idx = this.points.indexOf([x, y]);
		if(idx > -1) {
			this.points.splice(idx, 1);
			this.update(['points']);
		}
	}

	get isClosed() {
		return this.points[0] == this.points[this.points.length - 1];
	}

	close() {
		if(!this.isClosed) {
			this.points.push(this.points[0]);
			this.update(['points']);
		}
	}

}


let __planId = 0;

class Plan extends Datum {

	constructor() {
		super();
		this._id = ++__planId;
		this._name = `Plan${this.id}`;
		let boundary = this.boundary = new SimplePolygon();
		boundary.on('update', _bind(this, this.boundaryUpdate));
	}

	set name(n) {
		this._name = n;
		this.update();
	}

	get name() {
		return this._name;
	}

	get id() {
		return this._id;
	}

	boundaryUpdate(evt) {
		this.update(['boundary']);
	}

}


let __planManager = null;

class PlanManager extends EvtDisp {

	constructor() {
		super();
		if(__planManager)
			throw new Exception('PlanManager cannot be constructed. Use PlanManager.inst.');
		this.plans = new Map();
	}

	static get inst() {
		if(!__planManager)
			__planManager = new PlanManager();
		return __planManager;
	}

	newPlan() {
		let plan = null;
		do {
			plan = new Plan();
		} while(this.plans.has(plan.name));
		this.plans.set(plan.name, plan);
		plan.on('select', _bind(this, this.planSelected));
		plan.on('erase', _bind(this, this.planErased));
		this.dispatch('update');
		console.log(plan);
		return plan;
	}

	planErased(evt) {
		const name = evt.target.name;
		if(this.plans.has(name)) {
			this.plans.delete(name);
			this.dispatch('erase', {name: name});
			this.dispatch('update');
		}
	}

	planSelected(evt) {
		this.dispatch('select', {plan: evt.target});
	}

}

class Panel {

	constructor(app, sel) {
		this.app = app;
		this.panel = _qs(sel);
		this.hide();
	}

	show() {
		this.panel.style.display = 'block';
	}

	hide() {
		this.panel.style.display = 'show';
	}

	init() {}

	start() {}

	finish() {}

	cancel() {}

}


class PlansPanel extends Panel {

	constructor(app, sel) {
		super(app, sel);
		this.currentPlan = null;
		this.btnNewPlan = _listen(_qs('.btn_new_plan', this.panel), 'click', _bind(this, this.newPlan));
		this.btnPolygon = _listen(_qs('.btn_polygon', this.panel), 'click', _bind(this, this.polygon));
		this.btnSave = _listen(_qs('.btn_save', this.panel), 'click', _bind(this, this.finish));
		this.btnCancel = _listen(_qs('.btn_cancel', this.panel), 'click', _bind(this, this.cancel));
		this.txtPlanName = _listen(_qs('.plan_name', this.panel), 'change', _bind(this, this.formUpdate));
		this.btnEditBoundary = _listen(_qs('.btn_edit_boundary', this.panel), 'click', _bind(this, this.editBoundary));
		PlanManager.inst.on('select', _bind(this, this.planSelected));
		PlanManager.inst.on('update', _bind(this, this.plansUpdated));
	}

	editBoundary(evt) {
		evt.preventDefault();
		if(this.editingBoundary) {
			MapManager.inst.cursor = null;
			this.editingBoundary = false;
		} else {
			const mm = MapManager.inst;
			mm.cursor = 'crosshair';
			this.boundary = mm.getPolygon(this.currentPlan.name);
			this.editingBoundary = true;
		}
		console.log('edit boundary');
	}

	formUpdate(evt) {
		if(!this.currentPlan)
			return;
		switch(evt.target.name) {
		case 'name':
			this.currentPlan.name = evt.target.value;
			break;
		}
	}

	polygon(evt) {
		evt.preventDefault();
		console.log(this.name + ' polygon');
	}

	newPlan(evt) {
		evt.preventDefault();
		PlanManager.inst.newPlan().select();
	}

	plansUpdated(evt) {
		console.log('update');
		const lst = _qs('.plans_list', this.panel);
		console.log(this.panel, lst);
		_clear(lst);
		for(let [name, plan] of PlanManager.inst.plans) {
			let a = _text(_node('a'), plan.name);
			let node = _node('div');
			a.href = 'javascript:void(0);';
			a.dataset.name = plan.name;
			_listen(a, 'click', _bind(this, this.planClicked));
			node.appendChild(a);
			lst.appendChild(node);
		}
	}

	planClicked(evt) {
		evt.preventDefault();
		const plan = PlanManager.inst.plans.get(evt.target.dataset.name);
		plan.select();
	}

	planSelected(evt) {
		_qs('.plan_name', this.panel).value = evt.plan.name;
		if(this.currentPlan)
			this.currentPlan.off('update', _bind(this, this.planUpdated));
		this.currentPlan = evt.plan;
		this.currentPlan.on('update', _bind(this, this.planUpdated));
	}

	planUpdated(evt) {
		console.log('plan updated:', evt);
	}

	finish(evt) {
		evt.preventDefault();
		console.log(this.name + ' finish');
	}

	cancel(evt) {
		evt.preventDefault();		
		this.app.map.removePolygon(this.name);
		this.poly = null;
	}

}

let __mapManager = null;

class MapManager {

	constructor(app) {
		if(__mapManager)
			throw new Exception('MapManager cannot be constructed. Use MapManager.inst.');
		this.app = app;
		this.polygons = {};
		this.map = L.map('map').setView([48.42991, -123.34144], 15);
		this.baseMap = L.tileLayer('https://api.tiles.mapbox.com/v4/{id}/{z}/{x}/{y}.png?access_token={accessToken}', {
		    attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, <a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
    		maxZoom: 18,
    		id: 'mapbox.streets',
    		accessToken: MB_TOKEN
		}).addTo(this.map);
	}

	static create(app) {
		if(!__mapManager)
			__mapManager = new MapManager(app);
		return __mapManager;
	}

	static get inst() {
		if(!__mapManager)
			throw new Exception('MapManager must be created with MapManager.create().');
		return __mapManager;
	}

	set cursor(c) {
		const map = _qs('#map');
		map.style.cursor = c;
	}

	getPolygon(name, poly = []) {
		if(this.polygons.hasOwnProperty(name)) {
			return this.polygons[name];
		} else {
			return (this.polygons[name] = {
				poly: poly,
				layer: L.polygon(poly).addTo(this.map)
			});
		}
	}

	removePolygon(name) {
		if(this.polygons.hasOwnProperty(name)) {
			this.polygons[name].layer.removeFrom(this.map);
			delete this.polygons[name];
		}
	}

}

let __winManager = null;

class WindowManager {

	constructor(app) {
		if(__winManager)
			throw new Exception('WindowManager cannot be constructed. Use WindowManager.inst.');
		this._panels = new Map([
			['plans', new PlansPanel(app, '#panel_plans')]
		]);
		this.app = app;
		this.menu = _qs('#menu');
		_qsa('.menu_item', this.menu).forEach(item => {
			_listen(item, 'click', _bind(this, this.menuItemClick));
		});
		_listen(document.body, 'mousemove', _bind(this, this.trackMouse));
	}

	static get inst() {
		if(!__winManager)
			throw new Exception('WindowManager must be created with WindowManager.create().');
		return __winManager;
	}

	static create(app) {
		if(!__winManager)
			__winManager = new WindowManager(app);
		return __winManager;
	}

	menuItemClick(evt) {
		evt.preventDefault();
		const [action, param] = evt.target.dataset.action.split(':');
		this.app.doAction(action, param);
		return false;
	}

	trackMouse(evt) {
		evt.preventDefault();
		this.showMenu(evt.clientY < 40);
	}

	showMenu(show) {
		if(show && !this._menuShowing) {
			_qs('#menu').style.top = '0px';
			this._menuShowing = true;
		} else if(!show && this._menuShowing) {
			_qs('#menu').style.top = '-20px';
			this._menuShowing = false;
		}
	}

	showPanel(name) {
		for(let [k, v] of this._panels) {
			if(k == name) {
				v.show();
			} else {
				v.hide();
			}
		}
	}

}

let __app = null;

class App {

	constructor() {
		if(__app)
			throw new Exception('App cannot be constructed. Use App.inst.');
	}

	static get inst() {
		if(!__app)
			__app = new App();
		return __app;
	}

	start() {
		this.mapMan = MapManager.create(this);
		this.winMan = WindowManager.create(this);
		this.planMan = PlanManager.inst;
	}

	doAction(action, param) {
		console.log(action, param);
		if(action == 'panel') {
			this.winMan.showPanel(param);
		}
	}

}

window.main = function() {
	__app = new App();
	__app.start();
}
