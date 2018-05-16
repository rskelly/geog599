#!/usr/bin/env python

# http://scikit-learn.org/stable/modules/tree.html

from sklearn.datasets import load_iris
import pydotplus
import gdal
import random
import sys
import os
import struct
import csv 
import numpy as np
from math import isnan

# Run the sieve function on the output.
sieve_all = True

# Only these names will be produced, others filtered out.
matches = None #['T26', 'A32', 'J17', 'M20', 'T19', 'D3', 'Z2', 'I2', 'F3', 'X3']

class_names = ['none', 'water', 'wetland', 'upland', 'other', 'other2']

cart_config = {
	'type' : 'cart',
	'max_depth' : 4
}

kmeans_config = {
	'type' : 'kmeans',
	'clusters' : len(class_names)
},

svm_config = {
	'type' : 'svm',
	'C' : 1.,
	'kernel' : 'poly',
	'degree' : 3,
	'gamma' : 'auto',
	'coef0' : 0.,
	'probability' : False
}

# Template configuration object. Will be copied once for each input permutation.
config = {
	'steps' : {
		'create_samples' : False,
		'collect_samples' : True,
		'classify' : True,
		'validate' : True,
		'seive' : sieve_all,
	},
	'name' : None,
	'sample_file' : None,
	'sample_file_dir' : None, #'/Volumes/data1/rob_coop/coop_data/msc/layers/log_lake/classification/cart/',
	'validate_fraction' : 0.3,
	'sites_file' : 'sites.csv', #/Volumes/data1/rob_coop/coop_data/msc/layers/log_lake/classification/cart/e27_samples_strat_random_200.csv',
	'columns' : None,
	'class_names' : class_names,
	'files' : None,
	'outdir' : None,
	'config' : None
}

input_files = [
	('/media/rob/robdata/msc/layers/horn_river/masked/SlopeAspect_stddev_15_masked.tif',1),
	('/media/rob/robdata/msc/layers/horn_river/masked/SlopeGradient_median_15_masked.tif',1),
	('/media/rob/robdata/msc/layers/horn_river/tpi_50_median_5.tif',1),
	('/media/rob/robdata/msc/layers/horn_river/tpi_100_median_5.tif',1),
	('/media/rob/robdata/msc/layers/horn_river/tpi_500_median_5.tif',1),
	('/media/rob/robdata/msc/layers/horn_river/masked/Wetness_glcm_3_32_16_homo_mean_234678_masked.tif',1),
	('/media/rob/robdata/msc/layers/horn_river/masked/Wetness_glcm_3_32_16_homo_mean_all_masked.tif',1),
	('/media/rob/robdata/msc/layers/horn_river/masked/Wetness_glcm_3_32_16_homo_stddev_234678_masked.tif',1),
	('/media/rob/robdata/msc/layers/horn_river/masked/Wetness_glcm_3_32_16_homo_stddev_all_masked.tif',1)
]

# Tuples containing indices that cannot exist in the layer set together.
exclusions = [
	(2, 3, 4),
	(5,6),
	(7,8),
	(5,8),
	(6,7)
]

def combinations(lst, num):
	'''
	Create combinations of layers from the given list.
	There are num items in each combination.
	'''
	from itertools import combinations
	comb = list(combinations(lst, num))
	rem = set()
	for e in exclusions:
		# Get the filenames
		l = []
		for i in e:
			l.append(lst[i][0])

		# Run over the combinations.
		for i in range(len(comb)):
			c = comb[i]
			found = False
			for j in range(len(c)):
				if c[j][0] in l:
					if found:
						# An exclusion was already found; another has been found so add to the removal list.
						rem.add(i)
						break
					found = True
	# Create a new filtered list
	comb0 = []
	for i in range(len(comb)):
		if not i in rem:
			comb0.append(comb[i])
	return comb0

def configure(outdir, n, cls_config, select = None):
	'''
	Prepare configuration objects for each combination of layers.
	'''

	# Create the combinations
	from copy import copy
	combos = combinations(input_files, n)
	configs = []	

	# Create the output dir.
	try:
		os.makedirs(outdir)
	except:
		pass

	# Write the layer list to a csv file for each configuration.
	with open(os.path.join(outdir, 'configurations.csv'), 'w') as f:
		f.write('name,{}\n'.format(','.join(['file{}'.format(i) for i in range(n)])))
		i = 65
		m = 1
		for combo in combos:
			name = '{}{}'.format(chr(i), m)
			f.write('{},{}\n'.format(name, ','.join([x[0] for x in combo])))
			# If there's a selection list, filter out the
			# items not in the list.
			if not select or name in select:
				c = copy(config)
				c['name'] = name
				c['columns'] = [chr(j) for j in range(65, 65 + n)]
				c['files'] = combo
				c['outdir'] = outdir
				c['config'] = cls_config
				configs.append(c)
			i += 1
			if i > 90:
				i = 65
				m += 1
	return configs

def get_handles(files):
	'''
	Return open file handles and file information for each layer.
	Returns a tuple containing:
		a list of tuples: handle, transform, band number, nodata.
		a tuple containing the boundaries.
	'''
	handles = []
	bounds = [-99999999., -99999999., 99999999., 99999999.]
	for f, b in files:
		h = gdal.Open(f)
		t = h.GetGeoTransform()
		n = h.GetRasterBand(b).GetNoDataValue()
		handles.append([h, t, b, n])
		bounds[0] = max(bounds[0], t[0])
		bounds[3] = min(bounds[3], t[3])
		bounds[2] = min(bounds[2], t[0] + h.RasterXSize * t[1])
		bounds[1] = max(bounds[1], t[3] + h.RasterYSize * t[5])
	return handles, tuple(bounds)

def init_classifier(config):
	if config['type'] == 'cart':
		from sklearn import tree
		return tree.DecisionTreeClassifier(max_depth=config['max_depth'])
	elif config['type'] == 'kmeans':
		from sklearn import cluster
		return cluster.KMeans(n_clusters=config['clusters'])
	elif config['type'] == 'svm':
		from sklearn import svm
		return svm.SVC(C=config['C'], kernel=config['kernel'], degree=config['degree'], gamma=config['gamma'], coef0=config['coef0'], probability=config['probability'])
	else:
		raise Exception('No classification type given.')

def run(name, steps, sites_file, columns, class_names, files, sample_file, validate_fraction, outdir, sample_file_dir, config):
	'''
	Run the classification.
	'''

	# Attempt to create the output directory.
	try:
		os.makedirs(outdir)
	except Exception, e:
		print 'Failed to create directory,', outdir
		print e

	# Skip existing outputs.
	print 'Running', name
	if os.path.exists(os.path.join(outdir, name + '_result.tif')) or (matches and not name in matches):
		print 'Skipping', name
		return

	# Create the stats file if necessary.
	if not os.path.exists(os.path.join(outdir, 'correct.csv')):
		with open(os.path.join(outdir, 'correct.csv'), 'w') as f:
			f.write('name,correct\n');

	# Collect random samples from the study area. Will only keep the samples that have no nodata values.
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
					for handle, trans, band, nodata in handles:
						col = int((x - trans[0]) / trans[1])
						row = int((y - trans[3]) / trans[5])
						if col < 0 or row < 0 or col >= handle.RasterXSize or row >= handle.RasterYSize:
							continue;
						v = handle.GetRasterBand(band).ReadAsArray(col, row, 1, 1)[0,0]
						if v == nodata:
							v = -9999.
						values.append(v)
					if not -9999. in values:
						break
				f.write('{},{:.2f},{:.2f},{},{}\n'.format(i + 1, x, y, 0, 1))


	# Use sample locations from samples.csv to build a new sampling using
	# the inputs in the files list.
	if steps.get('collect_samples', False):

		handles, bounds = get_handles(files)
		if not sample_file:
			sample_file = os.path.join(outdir, name + '.csv')

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
					for handle, trans, band, nodata in handles:
						col = int((x - trans[0]) / trans[1])
						row = int((y - trans[3]) / trans[5])
						if col < 0 or row < 0 or col >= handle.RasterXSize or row >= handle.RasterYSize:
							valid = 0
							values.append(0)
						else:
							v = handle.GetRasterBand(band).ReadAsArray(col, row, 1, 1)[0,0]
							if v == nodata:
								v = -9999.
								valid = 0
							values.append(v)
					data = [id, x, y] + map(float, values) + [cls, valid]
					tpl = '{},{:.2f},{:.2f},' + '{:.6f},' * len(values) + '{},{}\n'
					g.write(tpl.format(*data))


	random_samples = None

	if steps.get('classify', False):

		from multiprocessing import Process, Lock
		from random import shuffle

		handles, bounds = get_handles(files)
		samples = []
		labels = []

		# Create an array with samples, and one with the class identier.
		# If the sample_file_dir override is specified, use a sample file from
		# a previous location. This only works if the file list is identical, and
		# the permutations are the same.
		if not sample_file:
			sample_file = os.path.join(outdir, name + '.csv')
			if sample_file_dir:
				sample_file = os.path.join(sample_file_dir, sample_file)

		print 'Loading samples from', sample_file
		with open(sample_file, 'rU') as f:
			db = csv.reader(f)
			db.next()
			random_samples = []
			for row in db:
				random_samples.append(row)
			shuffle(random_samples)
			for row in random_samples[int(len(random_samples) * validate_fraction):]:
				if int(row[-1]) == 1:
					samples.append(row[3 : 3 + len(columns)])
					labels.append(row[3 + len(columns)])

		# Add a sample an lable that represents none.
		samples.append([-9999.] * len(columns))
		labels.append(0)

		# Initialize the classifier.
		clf = init_classifier(config)
		clf = clf.fit(samples, labels)

		# Output the decision tree graph.
		if config['type'] == 'cart':
			from sklearn import tree
			with open('/tmp/tree.dot', 'w') as f:
				tree.export_graphviz(clf, out_file=f, class_names=class_names, feature_names=columns, node_ids=False)
			with open('/tmp/tree.dot', 'r') as f:
				graph = pydotplus.graph_from_dot_data(f.read())
				graph.write_pdf(os.path.join(outdir, name + "_cart.pdf"))

		# Get the handle, transform and band number of the first image.
		# This determines the properties of the output image.
		handle, trans, band, nodata = handles[0]
		cols = handle.RasterXSize
		rows = handle.RasterYSize

		# Creeate the output file.
		drv = gdal.GetDriverByName("GTiff")
		resrast = drv.Create(os.path.join(outdir, name + "_result.tif"), cols, rows, 1, gdal.GDT_Byte)
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
			print 'Doit'
			# Get the filehandles and minimum bounds.
			handles, bounds = get_handles(files)

			# Get the properties of the first image, the one used to define the output image.
			handle, trans, band, nodata = handles[0]

			# Iterate over the blocks.
			print 'Processing'
			while len(parts):
				print len(parts), 'left'
				x0, y0, x1, y1, wrow0, wrow1 = parts.pop(0)

				# Read the buffers from the source images
				#print 'collecting samples'
				samples = []
				y = y0 + trans[5] * 0.5
				cols = 0
				rows = 0
				while y > y1:
					x = x0 + trans[1] * 0.5
					cols = 0
					while x < x1:
						sample = []
						for h, t, b, n in handles:
							col = int((x - t[0]) / t[1])
							row = int((y - t[3]) / t[5])
							#print y, t[3], t[5], row
							try:
								v = h.GetRasterBand(b).ReadAsArray(col, row, 1, 1)[0, 0]
								if v == n:
									v = -9999.
								sample.append(v)
							except:
								sample.append(-9999.)

						samples.append(sample if not -9999. in sample else [-9999.] * len(sample))
						x += trans[1]
						cols += 1
					y += trans[5]
					rows += 1

				#print 'classifying -', len(samples), '*', len(samples[0]), 'samples'
				result = clf.predict(samples)
				
				row = int((y0 - trans[3]) / trans[5])
				#print 'writing -', cols, row, y0, y1
				result = np.array(result).reshape((len(samples) / cols, cols))
				lock.acquire()
				f = gdal.Open(os.path.join(outdir, name + '_result.tif'), gdal.GF_Write)
				f.GetRasterBand(1).WriteArray(result, 0, row)
				f = None
				lock.release()

		numThreads = 4
		print 'Starting', numThreads, 'threads'

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

		print 'Done'

	if steps.get('seive', False):
		print 'Running sieve'
		input = os.path.join(outdir, name + '_result.tif')
		output = os.path.join(outdir, name + '_result_sieve.tif')
		os.system('gdal_sieve.py -st 10 -8 {} {}'.format(input, output))

	if steps.get('validate'):
		print 'Validating'
		if not random_samples:
			raise Exception('No random samples were generated.')
		filename = os.path.join(outdir, name + '_result_sieve.tif')
		ds = gdal.Open(filename)
		band = ds.GetRasterBand(1)
		trans = ds.GetGeoTransform()
		results = []
		correct = 0
		for row in random_samples[:int(len(random_samples) * validate_fraction)]:
			if int(row[-1]) == 1:
				cls = int(row[-2])
				x, y = map(float, row[1:3])
				c = int((x - trans[0]) / trans[1])
				r = int((y - trans[3]) / trans[5])
				v = band.ReadAsArray(c, r, 1, 1)[0,0]
				results.append((row[0], x, y, v, cls))
				if v == cls:
					correct += 1
		with open(os.path.join(outdir, name + '_validate.csv'), 'w') as f:
			f.write('id,x,y,value,cls\n')
			for result in results:
				f.write(','.join(map(str, result)) + '\n')
		correct = float(correct) / len(results) if len(results) else -9999.
		with open(os.path.join(outdir, 'correct.csv'), 'a') as f:
			f.write('{},{:.2f}\n'.format(name, correct))
			print 'Correct:', correct

jobs = [
	configure('cart_2', 2, cart_config),
	configure('kmeans_2_all_cls', 2, kmeans_config),
	#configure('svm_2', 2, svm_config),
]

for job in jobs:
	for config in job:
		run(**config)