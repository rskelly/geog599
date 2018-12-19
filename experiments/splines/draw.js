function draw(pts, style) {
	let ctx = document.querySelector('#canv').getContext('2d');
	ctx.fillStyle = style;
	pts.forEach(pt => {
	 ctx.fillRect(pt.x - 2, pt.z - 2, 4, 4);
	});
}

function drawSegs(segs, style) {
  let ctx = document.querySelector('#canv').getContext('2d');
  ctx.strokeStyle = style;
  segs.forEach(seg => {
    ctx.beginPath();
    ctx.moveTo(seg[0].x, seg[0].z);
    ctx.lineTo(seg[1].x, seg[1].z);
    ctx.stroke();
  });
}

function drawLines(pts, style, offset = 0.0) {
  let ctx = document.querySelector('#canv').getContext('2d');
  ctx.beginPath();
  ctx.strokeStyle = style;
  ctx.moveTo(pts[0].x, pts[0].z);
  for(let i = 1; i < pts.length; ++i)
    ctx.lineTo(pts[i].x, pts[i].z + offset);
  ctx.stroke();
}
