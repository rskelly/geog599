#!/usr/bin/env python

# http://scikit-learn.org/stable/modules/tree.html

from sklearn import tree
from sklearn.datasets import load_iris
import pydotplus
import gdal
import random
import sys
import struct
import csv 
import numpy as np
from math import isnan

sample_file = 'samples_wet_grad.csv'

files = [
	('../../lm_wetness/Wetness_median_7.tif', 1),
	#('../../lm_wetness/SlopeAspect_median_7.tif', 1),
	('../../lm_wetness/SlopeGradient_median_7.tif', 1)
	#('../../lm_wetness/GLCM_Wetness/wetness_mean_masked_norm.tif', 1),
	#('../../lm_wetness/GLCM_SlopeGradient/gradient_mean_masked_norm.tif', 1)
	#('../../lidar_cov_canopy_4_5m.tif', 1)
]
columns = ['wetness', 'gradient']
class_names = ['invalid', 'upland', 'wetland', 'water', 'other']

handles = []
bounds = [-99999999., -99999999., 99999999., 99999999.]
for f, b in files:
	h = gdal.Open(f)
	t = h.GetGeoTransform()
	handles.append((h, t, b))
	bounds[0] = max(bounds[0], t[0])
	bounds[1] = max(bounds[1], t[3])
	bounds[2] = min(bounds[2], t[0] + h.RasterXSize * t[1])
	bounds[3] = min(bounds[3], t[3] + h.RasterYSize * t[5])

samples = []
labels = []

# Collect random samples from the study area.
# classes: 0 = invalid, 1 = upland, 2 = wetland, 3 = water, 4 = other
# valid: 1 = yes, 0 = no
if False:

	with open(sample_file, 'w') as f:
		f.write('id,x,y,' + ','.join(columns) + ',class,valid\n')
		for i in range(100):
			row = [i + 1, x, y]
			values = None
			while True:
				x = bounds[0] + random.random() * (bounds[2] - bounds[0])
				y = bounds[1] + random.random() * (bounds[3] - bounds[1])
				values = []
				for handle, trans, band in handles:
					col = int((x - bounds[0]) / trans[1])
					row = int((y - bounds[3]) / trans[5])
					values.append(struct.unpack('f', handle.GetRasterBand(band).ReadRaster(col, row, 1, 1))[0])
				if not -9999. in values:
					break
			data = row + values + [0, 1]
			tpl = '{},{:.2f},{:.2f},' + '{:.6},' * len(values) + '{},{}\n'
			samples.append(data[3 : 3 + len(columns)])
			labels.append(data[5])
			f.write(tpl.format(*data))


# Use sample locations from samples.csv to build a new sampling using
# the intputs in the files list.
if True:

	with open(sample_file, 'w') as g:
		with open('samples.csv', 'rU') as f:
			db = csv.reader(f)
			head = db.next()
			g.write(','.join(head[:3] + columns + head[-2:]) + '\n')
			for dbrow in db:
				x = float(dbrow[1])
				y = float(dbrow[2])
				values = []
				for handle, trans, band in handles:
					col = int((x - trans[0]) / trans[1])
					row = int((y - trans[3]) / trans[5])
					values.append(struct.unpack('f', handle.GetRasterBand(band).ReadRaster(col, row, 1, 1))[0])
				valid = 1 if not -9999. in values else 0
				data = dbrow[0:1] + map(float, dbrow[1:3]) + values + [dbrow[-2], valid]
				tpl = '{},{:.2f},{:.2f},' + '{:.6f},' * len(values) + '{},{}\n'
				samples.append(data[3 : 3 + len(columns)])
				labels.append(data[5])
				g.write(tpl.format(*data))


# Fit the model and run on raster data.
if True: 

	if not len(samples):
		with open(sample_file, 'rU') as f:
			db = csv.reader(f)
			db.next()
			for row in db:
				if int(row[-1]) == 1:
					samples.append(row[3 : 3 + len(columns)])
					labels.append(row[3 + len(columns)])

	samples.append([-9999.] * len(columns))
	labels.append(0)

	feature_names = columns

	clf = tree.DecisionTreeClassifier(max_depth=3)
	clf = clf.fit(samples, labels)

	dot_data = tree.export_graphviz(clf, out_file=None, class_names=class_names, feature_names=feature_names, node_ids=False)
	graph = pydotplus.graph_from_dot_data(dot_data)
	graph.write_pdf("test.pdf")

	trans = handles[0][1]
	proj = handles[0][0].GetProjectionRef()
	cols = handles[0][0].RasterXSize
	rows = handles[0][0].RasterYSize

	drv = gdal.GetDriverByName("GTiff")
	resrast = drv.Create("result.tif", cols, rows, 1, gdal.GDT_Byte)
	resrast.SetGeoTransform(trans)
	resrast.SetProjection(proj)
	resrast = None

	from multiprocessing import Process

	steps = []
	step = 256
	for row0 in range(0, rows, step):
		row1 = min(row0 + step, rows)
		steps.append((row0, row1))

	def doit(steps, files):

		handles = []
		for f, b in files:
			h = gdal.Open(f)
			t = h.GetGeoTransform()
			handles.append([h, t, b, None])

		while len(steps):
			row0, row1 = steps.pop(0)
			print 'row', row0, 'to', row1

			print 'reading buffers'
			samples = []
			for i in range(len(handles)):
				h, t, b, a = handles[i]
				handles[i][3] = h.GetRasterBand(b).ReadAsArray(0, row0, h.RasterXSize, row1 - row0)

			print 'processing'
			for r in range(row0, row1):
				for c in range(cols):
					x = c * trans[1] + trans[0] + trans[1] * 0.5
					y = r * trans[5] + trans[3] + trans[5] * 0.5
					row = []
					hi = 0
					for h, t, b, a in handles:
						cc = int((x - t[0]) / t[1])
						rr = int((y - t[3]) / t[5])
						if cc < 0 or rr < 0 or cc >= h.RasterXSize or rr >= h.RasterYSize:
							row.append(-9999.)
						else:
							v = a[rr - row0, cc]
							row.append(v if not (v is None or isnan(v) or v > 999999. or v < -999999.) else -9999.)
						hi += 1
					samples.append(row)

			print 'classifying'
			result = clf.predict(samples)
			
			print 'writing'
			result = np.array(result).reshape((row1 - row0, cols))
			f = gdal.Open('result.tif', gdal.GF_Write)
			f.GetRasterBand(1).WriteArray(result, 0, row0)

	numThreads = 8
	print 'starting', numThreads, 'threads'

	threads = []
	for i in range(numThreads):
		a = i * int(len(steps) / numThreads)
		b = a + int(len(steps) / numThreads)
		print a, b
		t = Process(target=doit, args=(steps[a:b], files))
		threads.append(t)
		t.start()

	for i in range(numThreads):
		threads[i].join()

	print 'done'