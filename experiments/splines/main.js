
const filenames = [/*'mt_doug_1.txt',*/ 'mt_doug_2.txt', 'mt_doug_3.txt'];
const pc = new PointCloud();

let drawPc;

let speed = 4.0;
let altitude = 100;
let angle = 25.0;
let divergence = 0.2;
let startTime;
let running = false;
let alpha = 0.5;
let binSize = 10.0;

let animY0 = 0;
let animY1 = 0;

const stylePoint = '#aaffaa';

function update(evt) {
  if(evt)
    evt.preventDefault();
  angle = parseFloat(document.querySelector('#angle').value);
  divergence = parseFloat(document.querySelector('#divergence').value);
  altitude = parseFloat(document.querySelector('#altitude').value);
  speed = parseFloat(document.querySelector('#speed').value);
  alpha = parseFloat(document.querySelector('#alpha').value);
  binSize = parseFloat(document.querySelector('#binSize').value);
}

function loadFile(evt) {
  evt.preventDefault();
  pc.load(evt.target.selectedOptions[0].text, (status, msg) => {
    if(status)
      pc.transform();
    document.querySelector('#loading').innerHTML = msg;
  });
}

let accum = new PointCloud();
let tick = 0;
let pcframe = null;

function drawPointCloud() {
  const canv = document.querySelector('#canv');
  const ctx = canv.getContext('2d');
  const screen = new Bounds(0, 0, 0, canv.width, 0, canv.height);
  const bounds = pc.bounds.clone();

  let spc = new PointCloud(pc.points);
  spc.scale(screen, bounds);

  ctx.clearRect(0, 0, screen.length, screen.height);

  // Draw the whole cloud lightly.
  ctx.fillStyle = '#dddddd';
  spc.points.forEach(pt => {
    ctx.fillRect(pt.y - 2, screen.zmax - pt.z - 2, 4, 4);
  });

  pcframe = ctx.getImageData(0, 0, screen.length, screen.height);
}

function animate(timestamp) {

  tick += 100;

  if(startTime == null) {
    // If this is the first frame, get the time and reschedule.
    startTime = tick;
    requestAnimationFrame(animate);
    return;
  }

  const t = (tick - startTime) / 1000.0;

  const canv = document.querySelector('#canv');
  const ctx = canv.getContext('2d');

  const screen = new Bounds(0, 0, 0, canv.width, 0, canv.height);
  const bounds = pc.bounds.clone();

  // The y (left to right) position is dictated by time.
  let y = t * speed;
  let z = altitude;

  // Create a point to represent the UAV and a point cloud containing
  // only the UAV (for display).
  let uav = new Point(0, y, z);
  let uavpc = new PointCloud([uav]);
  bounds.extend(uav);

  // The slice of the point cloud that will be displayed.
  let slice = pc.sliceYRay(y, z, -angle * Math.PI / 180, divergence * Math.PI / 180);

  // Remove points behind the craft.
  accum.filterY(y - binSize);
// Collect the accumulated points.
  slice.points.forEach(pt => {
    accum.addPoint(pt.clone());
  });
  
  // Get the convex hull of the accumulated points.
  let hull = accum.getHull(binSize);

  // Get the spline throught the binned heights.
  let spline = getCRSplines(hull.points, alpha);

  let tslice = hull.sliceSeg(y, 4);
  let traj = null;
  if(tslice.length == 4) {
    const t0 = y;
    const t1 = crTJ(t0, tslice[0], tslice[1], alpha);
    const t2 = crTJ(t1, tslice[1], tslice[2], alpha);
    let t = t1 + (t2 - t1) / 2;//(y - tslice[1].y);// / (tslice[2].y - tslice[1].y);
    traj = new PointCloud([catmullRomSplineT(t, tslice[0], tslice[1], tslice[2], tslice[3], alpha)]);
  }

  // Scale point clouds for display.
  slice.scale(screen, bounds);
  uavpc.scale(screen, bounds);
  hull.scale(screen, bounds);
  Point.scale(spline, screen, bounds);
  if(traj)
    traj.scale(screen, bounds);

  // Draw the point cloud background.
  ctx.putImageData(pcframe, 0, 0);

  // Draw the slice points.
  ctx.fillStyle = stylePoint;
  slice.points.forEach(pt => {
    ctx.fillRect(pt.y - 2, screen.zmax - pt.z - 2, 4, 4);
  });

  // Draw the hull points.
  ctx.fillStyle = 'green';
  hull.points.forEach(pt => {
    ctx.fillRect(pt.y - 2, screen.zmax - pt.z - 2, 4, 4);
  });

  // Draw the UAV.
  let uavpt = uavpc.points[0];
  ctx.fillStyle = 'black';
  ctx.fillRect(uavpt.y - 3, screen.zmax - uavpt.z - 3, 6, 6);

  // Draw the height spline
  if(spline.length) {
    ctx.strokeStyle = 'blue';
    ctx.beginPath();
    ctx.moveTo(spline[0].y, spline[0].z);
    for(let i = 1; i < spline.length; ++i)
      ctx.lineTo(spline[i].y, screen.zmax - spline[i].z);
    ctx.stroke();
  }

  if(traj) {
    ctx.strokeStyle = 'black';
    ctx.fillStyle = 'fuscia';
    ctx.fillRect(traj.points[0].y - 3, screen.zmax - traj.points[0].z - 3, 6, 6);
  }

  let lookAhead = new PointCloud([
    new Point(0, y + speed, altitude), 
    new Point(0, y + speed * 2, altitude), 
    new Point(0, y + speed * 3, altitude),
    new Point(0, y + speed * 4, altitude),
    new Point(0, y + speed * 5, altitude),
    new Point(0, y + speed * 6, altitude),
    new Point(0, y + speed * 7, altitude),
    new Point(0, y + speed * 8, altitude),
    new Point(0, y + speed * 9, altitude),
    new Point(0, y + speed * 10, altitude)
  ]);
  lookAhead.scale(screen, bounds);
  ctx.strokeStyle = '#ff00ff22';
  lookAhead.points.forEach(pt => {
    ctx.beginPath();
    ctx.moveTo(pt.y - 2, screen.zmax);
    ctx.lineTo(pt.y - 2, screen.zmin);
    ctx.stroke();
  });

  if(running)
    requestAnimationFrame(animate);
}

function start(evt) {
  evt.preventDefault();
  update();
  startTime = null;
  pcframe = null;
  running = true;
  accum.reset();
  drawPointCloud();
  requestAnimationFrame(animate);
}

function stop(evt) {
  evt.preventDefault();
  running = false;
}

function init() {
  let w = document.body.clientWidth;
  let h = 600;
  let canv = document.querySelector('#canv');
  canv.width = w;
  canv.height = h;
  let sel = document.querySelector('#filename');
  sel.add(new Option('', '- Select One -'));
  let i = 0;
  filenames.forEach(f => {
    sel.add(new Option(f, ++i));
  });
  sel.addEventListener('change', loadFile);

  document.querySelector('#altitude').value = altitude;
  document.querySelector('#speed').value = speed;
  document.querySelector('#binSize').value = binSize;
  document.querySelector('#angle').value = angle;
  document.querySelector('#divergence').value = divergence;

}
