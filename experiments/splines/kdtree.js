// stolen from https://bost.ocks.org/mike/shuffle/
function shuffle(array) {
  var m = array.length, t, i;

  // While there remain elements to shuffle…
  while (m) {

    // Pick a remaining element…
    i = Math.floor(Math.random() * m--);

    // And swap it with the current element.
    t = array[m];
    array[m] = array[i];
    array[i] = t;
  }

  return array;
}

const _pow = Math.pow;

function _dist(a, b, dims) {
	let d = 0;
	for(let i = 0; i < dims; ++i)
		d += _pow(a.coord(i) - b.coord(i), 2);
	return d;
}

function _coordSort(dim) {
	return function(a, b) {
		return a.coord(dim) < b.coord(dim);
	};
}

function _distSort(item) {
	return function(a, b) {
		return _dist(a, item) < _dist(b, item);
	};
}

function _buildKDTRee(items, depth, dims, parent = null) {
	let dim = depth % dims;
	items.sort(_coordSort(dim));
	let medIdx = parseInt(items.length / 2);
	let median = items[med];
	let node = {
		item: median,
		parent: parent
	};
	if(items.length > 1) {
		node.a = _buildKDTree(items.slice(0, med), depth + 1, dims, node);
		node.b = _buildKDTree(items.slice(med, items.length), depth + 1, dims, node);
	}
	return node;
}


function _knn(item, parent, node, k, depth, dims, nearest = []) {
	if(node == null)
		return nearest;
	let b = node;
	let t = depth;
	while(true) {
		if(!b.a) break;
		if(item.coord(t % dims) < b.item.coord(t % dims)) {
			b = b.a;
		} else {
			b = b.b;
		}
		++t;
	}
	let d = _dist(item, b);
	while((b = b.parent) != null) {

	}
	let dim = depth % dims;
	let db = _dist(item, node);
	let best = node;
	let d;
	if(node.a && (d = _dist(item, node.a.item)) < db) {
		db = d;
	} else if(node.b && (d = _dist(item, node.b.item)) < db) {
		db = d;
	}
	nearest.push(node.item);
	if(item.coord(dim) <= node.item.coord(dim)) {
		_knn(item, node.a, depth + 1, dims, nearest);
	} else {
		_knn(item, node.b, depth + 1, dims, nearest);
	}
	return nearest.sort(_distSort(item)).slice(0, k);
}

class KDTree {
	
	constructor(dims = 3) {
		this._dims = dims;
		this._needsBuild = true;
		this._items = [];
	}

	add(item) {
		if(!item.coord)
			throw new Error('An item must have a coord(index) method.');
		this._items.push(item);
		this._needsBuild = true;
	}

	knn(item, k = 1) {
		if(!item.coord)
			throw new Error('An item must have a coord(index) method.');
		if(this._needsBuild)
			this._build();
		return _knn(this._node, item, k);
	}

	_build() {
		this._node = _buildKDTRee(this._items, 0, this._dims);
		this.needsBuild = false;
	}
}