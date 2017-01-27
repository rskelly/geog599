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
from subprocess import Popen

configs = []

sieve_all = True

config = {
	'steps' : {
		'create_samples' : False,
		'collect_samples' : True,
		'classify' : True,
		'seive' : sieve_all,
	},
	'name' : 'slope_mean_norm_wetrough_mean10',
	'sites_file' : 'sites_random_manual.csv',
	'max_depth' : 4,
	'columns' : ['gradient', 'wetness_roughness'],
	'class_names' : ['none', 'water', 'wetland', 'upland'],
	'files' : [
		('../../lm_wetness/GLCM_SlopeGradient/gradient_mean_masked_norm.tif', 1),
		('wetness_roughness_mean10.tif', 1),
	],
}
#configs.append(config)

config = {
	'steps' : {
		'create_samples' : False,
		'collect_samples' : True,
		'classify' : True,
		'seive' : sieve_all,
	},
	'name' : 'slope_mean_norm_wet_mean_norm',
	'sites_file' : 'sites_random_manual.csv',
	'max_depth' : 4,
	'columns' : ['gradient', 'wetness'],
	'class_names' : ['none', 'water', 'wetland', 'upland'],
	'files' : [
		('../../lm_wetness/GLCM_SlopeGradient/gradient_mean_masked_norm.tif', 1),
		('../../lm_wetness/GLCM_Wetness/wetness_mean_masked_norm.tif', 1),
	],
}
#configs.append(config)

config = {
	'steps' : {
		'create_samples' : False,
		'collect_samples' : True,
		'classify' : True,
		'seive' : sieve_all,
	},
	'name' : 'slope_mean_wet_mean_aspect_std',
	'sites_file' : 'sites_random_manual.csv',
	'max_depth' : 4,
	'columns' : ['gradient', 'wetness', 'aspect'],
	'class_names' : ['none', 'water', 'wetland', 'upland'],
	'files' : [
		('../../lm_wetness/GLCM_SlopeGradient/gradient_mean_masked_norm.tif', 1),
		('../../lm_wetness/GLCM_Wetness/wetness_mean_masked_norm.tif', 1),
		('../../lm_wetness/GLCM_SlopeAspect/image_standard_deviation.tif', 1),
	],
}
configs.append(config)

config = {
	'steps' : {
		'create_samples' : False,
		'collect_samples' : True,
		'classify' : True,
		'seive' : sieve_all,
	},
	'name' : 'slope_mean_wetrough_mean10_aspect_std',
	'sites_file' : 'sites_random_manual.csv',
	'max_depth' : 4,
	'columns' : ['gradient', 'wet_rough', 'aspect'],
	'class_names' : ['none', 'water', 'wetland', 'upland'],
	'files' : [
		('../../lm_wetness/GLCM_SlopeGradient/gradient_mean_masked_norm.tif', 1),
		('wetness_roughness_mean10.tif', 1),
		('../../lm_wetness/GLCM_SlopeAspect/image_standard_deviation.tif', 1),
	],
}
configs.append(config)

config = {
	'steps' : {
		'create_samples' : False,
		'collect_samples' : True,
		'classify' : True,
		'seive' : sieve_all,
	},
	'name' : 'slope_mean_aspect_std',
	'sites_file' : 'sites_random_manual.csv',
	'max_depth' : 4,
	'columns' : ['gradient', 'aspect'],
	'class_names' : ['none', 'water', 'wetland', 'upland'],
	'files' : [
		('../../lm_wetness/GLCM_SlopeGradient/gradient_mean_masked_norm.tif', 1),
		('../../lm_wetness/GLCM_SlopeAspect/image_standard_deviation.tif', 1),
	],
}
configs.append(config)

config = {
	'steps' : {
		'create_samples' : False,
		'collect_samples' : True,
		'classify' : True,
		'seive' : sieve_all,
	},
	'name' : 'slope_mean_wetrough_mean10',
	'sites_file' : 'sites_random_manual.csv',
	'max_depth' : 4,
	'columns' : ['wetness_roughness', 'gradient'],
	'class_names' : ['none', 'water', 'wetland', 'upland'],
	'files' : [
		('wetness_roughness_mean10.tif', 1),
		('../../lm_wetness/GLCM_SlopeGradient/image_standard_deviation.tif', 1),
	],
}
configs.append(config)

config = {
	'steps' : {
		'create_samples' : False,
		'collect_samples' : True,
		'classify' : True,
		'seive' : sieve_all,
	},
	'name' : 'b200_slope_mean_wetrough_mean10',
	'sites_file' : 'sites_random_manual.csv',
	'max_depth' : 4,
	'columns' : ['gradient', 'wetness', 'b200'],
	'class_names' : ['none', 'water', 'wetland', 'upland'],
	'files' : [
		('../../lm_wetness/GLCM_SlopeGradient/image_mean.tif', 1),
		('wetness_roughness_mean10.tif', 1),
		('../../hyper/hyperspectral_mosaic_2_25m.tif', 200),
	],
}
#configs.append(config)

config = {
	'steps' : {
		'create_samples' : False,
		'collect_samples' : True,
		'classify' : True,
		'seive' : sieve_all,
	},
	'name' : 'water_hs',
	'sites_file' : 'sites_random_manual.csv',
	'max_depth' : 4,
	'columns' : ['b176', 'b91', 'b56', 'b20'],
	'class_names' : ['none', 'water', 'wetland', 'upland'],
	'files' : [
		('../../hyper/hyperspectral_mosaic_2_25m.tif', 176),
		('../../hyper/hyperspectral_mosaic_2_25m.tif', 91),
		('../../hyper/hyperspectral_mosaic_2_25m.tif', 56),
		('../../hyper/hyperspectral_mosaic_2_25m.tif', 20),
	],
}
configs.append(config)



# Files to use for sampling.
_files = [
	#('../../hyper/hyperspectral_mosaic_2_25m.tif', 200),
	#('../../lm_wetness/SlopeGradient_median_7.tif', 1),
	#('../../lm_wetness/Wetness_median_7.tif', 1),
	#('../../lm_wetness/SlopeAspect_median_7.tif', 1),
	#('../../lm_wetness/Wetness_median_21.tif', 1),
	#('../../lm_wetness/SlopeAspect_median_21.tif', 1),
	#('../../lm_wetness/SlopeGradient_median_21.tif', 1)
	#('../../lm_wetness/GLCM_SlopeGradient/image_standard_deviation.tif', 1),
	#('../../lm_wetness/GLCM_Wetness/image_standard_deviation.tif', 1),
	('../../lm_wetness/GLCM_SlopeGradient/gradient_mean_masked_norm.tif', 1),
	('../../lm_wetness/GLCM_Wetness/wetness_mean_masked_norm.tif', 1),
	('../../lm_wetness/GLCM_SlopeAspect/image_standard_deviation.tif', 1),
	#('../../lidar_cov_canopy_4_5m.tif', 1),
]


def get_handles(files):
	handles = []
	bounds = [-99999999., -99999999., 99999999., 99999999.]
	for f, b in files:
		h = gdal.Open(f)
		t = h.GetGeoTransform()
		handles.append([h, t, b])
		bounds[0] = max(bounds[0], t[0])
		bounds[3] = min(bounds[3], t[3])
		bounds[2] = min(bounds[2], t[0] + h.RasterXSize * t[1])
		bounds[1] = max(bounds[1], t[3] + h.RasterYSize * t[5])
	return handles, tuple(bounds)

def run(name, steps, sites_file, max_depth, columns, class_names, files):

	# Collect random samples from the study area.
	# classes: 0 = invalid, 1 = upland, 2 = wetland, 3 = water, 4 = other
	# valid: 1 = yes, 0 = no
	if steps.get('create_samples', False):

		handles, bounds = get_handles(files)
		samples = 200

		with open(sites_file, 'w') as f:
			f.write('id,x,y,class,valid\n')
			for i in range(samples):
				values = None
				x = 0
				y = 0
				while True:
					x = bounds[0] + random.random() * (bounds[2] - bounds[0])
					y = bounds[1] + random.random() * (bounds[3] - bounds[1])
					values = []
					for handle, trans, band in handles:
						col = int((x - trans[0]) / trans[1])
						row = int((y - trans[3]) / trans[5])
						if col < 0 or row < 0 or col >= handle.RasterXSize or row >= handle.RasterYSize:
							continue;
						v = handle.GetRasterBand(band).ReadAsArray(col, row, 1, 1)[0,0]
						values.append(v)
					if not -9999. in values:
						break
				f.write('{},{:.2f},{:.2f},{},{}\n'.format(i + 1, x, y, 0, 1))


	# Use sample locations from samples.csv to build a new sampling using
	# the intputs in the files list.
	if steps.get('collect_samples', False):

		handles, bounds = get_handles(files)
		sample_file = name + '.csv'

		with open(sample_file, 'w') as g:
			with open(sites_file, 'rU') as f:
				db = csv.reader(f)
				head = db.next()
				g.write(','.join(head[:3] + columns + head[-2:]) + '\n')
				for dbrow in db:
					id = int(dbrow[0])
					x = float(dbrow[1])
					y = float(dbrow[2])
					cls = int(dbrow[3])
					values = []
					valid = 1
					for handle, trans, band in handles:
						col = int((x - trans[0]) / trans[1])
						row = int((y - trans[3]) / trans[5])
						if col < 0 or row < 0 or col >= handle.RasterXSize or row >= handle.RasterYSize:
							valid = 0
							values.append(0)
						else:
							v = handle.GetRasterBand(band).ReadAsArray(col, row, 1, 1)[0,0]
							values.append(v)
							if v == handle.GetRasterBand(band).GetNoDataValue():
								valid = 0
					data = [id, x, y] + map(float, values) + [cls, valid]
					tpl = '{},{:.2f},{:.2f},' + '{:.6f},' * len(values) + '{},{}\n'
					g.write(tpl.format(*data))


	if steps.get('classify', False):

		from multiprocessing import Process, Lock

		handles, bounds = get_handles(files)
		samples = []
		labels = []

		# Create an array with samples, and one with the class identier.
		sample_file = name + '.csv'
		print 'Loading samples from', sample_file
		with open(sample_file, 'rU') as f:
			db = csv.reader(f)
			db.next()
			for row in db:
				if int(row[-1]) == 1:
					samples.append(row[3 : 3 + len(columns)])
					labels.append(row[3 + len(columns)])

		# Add a sample an lable that represents none.
		samples.append([-9999.] * len(columns))
		labels.append(0)

		# Initialize the classifier.
		clf = tree.DecisionTreeClassifier(max_depth=max_depth)
		clf = clf.fit(samples, labels)

		# Output the decision tree graph.
		dot_data = tree.export_graphviz(clf, out_file=None, class_names=class_names, feature_names=columns, node_ids=False)
		graph = pydotplus.graph_from_dot_data(dot_data)
		graph.write_pdf(name + "_cart.pdf")

		# Get the handle, transform and band number of the first image.
		# This determines the properties of the output image.
		handle, trans, band = handles[0]
		cols = handle.RasterXSize
		rows = handle.RasterYSize

		# Creeate the output file.
		drv = gdal.GetDriverByName("GTiff")
		resrast = drv.Create(name + "_result.tif", cols, rows, 1, gdal.GDT_Byte)
		resrast.SetGeoTransform(trans)
		resrast.SetProjection(handle.GetProjectionRef())
		resrast = None
		print 'Created output file', name + "_result.tif", cols, rows

		# Generate a list of work blocks
		parts = []
		step = 32
		for row0 in range(0, rows, step):
			row1 = min(row0 + step, rows)
			x0 = bounds[0]
			x1 = bounds[2]
			y0 = bounds[3] + row0 * trans[5]
			y1 = bounds[3] + row1 * trans[5]
			parts.append((x0, y0, x1, y1, row0, row1))

		# Parallizable sampling/classification function
		def doit(parts, files, lock):

			# Get the filehandles and minimum bounds.
			handles, bounds = get_handles(files)

			# Get the properties of the first image, the one used to define the output image.
			handle, trans, band = handles[0]

			# Iterate over the blocks.
			while len(parts):
				x0, y0, x1, y1, wrow0, wrow1 = parts.pop(0)

				# Read the buffers from the source images
				print 'collecting samples'
				samples = []
				y = y0 + trans[5] * 0.5
				cols = 0
				rows = 0
				while y > y1:
					x = x0 + trans[1] * 0.5
					cols = 0
					while x < x1:
						sample = []
						for h, t, b in handles:
							col = int((x - t[0]) / t[1])
							row = int((y - t[3]) / t[5])
							#print y, t[3], t[5], row
							try:
								v = h.GetRasterBand(b).ReadAsArray(col, row, 1, 1)[0, 0]
								if v == h.GetRasterBand(b).GetNoDataValue():
									v = -9999.
								sample.append(v)
							except:
								sample.append(-9999.)

						samples.append(sample if not -9999. in sample else [-9999.] * len(sample))
						x += trans[1]
						cols += 1
					y += trans[5]
					rows += 1

				print 'classifying -', len(samples), '*', len(samples[0]), 'samples'
				result = clf.predict(samples)
				
				row = int((y0 - trans[3]) / trans[5])
				print 'writing -', cols, row, y0, y1
				result = np.array(result).reshape((len(samples) / cols, cols))
				lock.acquire()
				f = gdal.Open(name + '_result.tif', gdal.GF_Write)
				f.GetRasterBand(1).WriteArray(result, 0, row)
				f = None
				lock.release()

		numThreads = 9
		print 'starting', numThreads, 'threads'

		threads = []
		lock = Lock()
		for i in range(numThreads):
			a = i * int(len(parts) / numThreads)
			b = a + int(len(parts) / numThreads)
			t = Process(target=doit, args=(parts[a:b], files, lock))
			threads.append(t)
			t.start()

		for i in range(numThreads):
			threads[i].join()

		print 'done'

	if steps.get('seive', False):
		print 'running sieve'
		input = name + '_result.tif'
		output = name + '_result_sieve.tif'
		Popen(['gdal_sieve.py', '-st', '10', '-8', input, output]).wait()

for config in configs:
	run(**config)