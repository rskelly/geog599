#!/usr/bin/env python

'''
This script reads values from a spreadsheet and clusters them using K-Means. The 
spreadsheet is produced by selecting pixel values from one or more rasters using
a classified mask raster. Each row contains the mask's class, the filename of each
input raster and the values associated with the selected band(s) from that raster.

Each 'job' is runs the clustering algorithm on a set of columns from the spreadsheet.
The properties of a job object are:
 - filename - The filename of the source spreadsheet.
 - columns  - The indices of the columns in the spreadsheet (starting with 0)
 - k        - The number of clusters.
 - cls      - The class to select. 
 - outfile  - The output file; a CSV.
 '''
 
from sklearn.cluster import KMeans
import numpy as np
import sys
import os
import csv
import gdal
from math import isnan

csvfile = 'all.csv'
rasters = ['/media/rob/robdata/msc/layers/horn_river/final/biometrics_20m_allcls_lmtools.tif', 
			'/media/rob/robdata/msc/layers/horn_river/final/WEST_DEM_2m_TIFF_saga_vdcn_001ha_20m.tif', 
			'/media/rob/robdata/msc/layers/horn_river/masked/WEST_DEM_2m_TIFF_sagatwi.tif'
			]
clsraster = '/media/rob/robdata/msc/layers/horn_river/final/kmeans_2/H1_result_reclass_20m.tif'

jobs = [
	{
		'filename' : csvfile,
		'columns' : (15,),
		'k' : 4,
		'cls' : 2,
		'outfile' : 'vdcn_001ha_cls2.csv',
	},
	{
		'filename' : csvfile,
		'columns' : (15,),
		'k' : 4,
		'cls' : 1,
		'outfile' : 'vdcn_001ha_cls1.csv',
	},
	{
		'filename' : csvfile,
		'columns' : (18,),
		'k' : 4,
		'cls' : 1,
		'outfile' : 'sagatwi_cls1.csv',
	},
	{
		'filename' : csvfile,
		'columns' : (18,),
		'k' : 4,
		'cls' : 2,
		'outfile' : 'sagatwi_cls2.csv',
	},
	# {
	# 	'filename' : csvfile,
	# 	'columns' : (4,5,6,7,8,10,),
	# 	'k' : 4,
	# 	'cls' : 2,
	# 	'outfile' : 'b.csv',
	# },
	# {
	# 	'filename' : csvfile,
	# 	'columns' : (4,5,6,7,8,10,12,),
	# 	'k' : 4,
	# 	'cls' : 2,
	# 	'outfile' : 'c.csv',
	# },
	# {
	# 	'filename' : csvfile,
	# 	'columns' : (12,),
	# 	'k' : 4,
	# 	'cls' : 1,
	# 	'outfile' : 'd.csv',
	# },
	# {
	# 	'filename' : csvfile,
	# 	'columns' : (4,5,6,7,8,10,),
	# 	'k' : 4,
	# 	'cls' : 1,
	# 	'outfile' : 'e.csv',
	# },
	# {
	# 	'filename' : csvfile,
	# 	'columns' : (4,5,6,7,8,10,12,),
	# 	'k' : 4,
	# 	'cls' : 1,
	# 	'outfile' : 'f.csv',
	# },
]

def generate_all(rasters, clsraster, csvfile):

	# Open the class raster.
	clsds = gdal.Open(clsraster)
	clstrans = clsds.GetGeoTransform()

	# Open the other raster files; get the transform, nodata and filename
	rds = []
	for rast in rasters:
		ds = gdal.Open(rast)
		rds.append((ds, ds.GetGeoTransform(), ds.GetRasterBand(1).GetNoDataValue(), rast))

	# Prepare the csv header using the datasource bands/filenames
	csvhead = ['cls', 'x', 'y']
	for ds, trans, nd, rast in rds:
		csvhead.append(rast)
		for b in range(1, ds.RasterCount + 1):
			csvhead.append('b{n}'.format(n=b))

	# Class raster nodata.
	cnd = clsds.GetRasterBand(1).GetNoDataValue()

	# Open output file.
	with open(csvfile, 'w') as f:

		# Write header.
		f.write(','.join(csvhead) + '\n')

		# Iterate over cols/rows in class raster.
		for row in range(clsds.RasterYSize):
			for col in range(clsds.RasterXSize):
				
				cx = col * clstrans[1] + clstrans[0]
				cy = row * clstrans[5] + clstrans[3]

				# Get class; skip if nodata.
				cls = clsds.GetRasterBand(1).ReadAsArray(col, row, 1, 1)[0,0]
				if cls == cnd:
					continue

				csvrow = [cls, cx, cy]

				keepRow = True

				# Iterate over input rasters.
				for ds, trans, nd, rast in rds:

					# Add the filename column to the row.
					csvrow.append(rast)

					dc = (cx - trans[0]) / trans[1]
					dr = (cy - trans[3]) / trans[5]

					# For each band, get the value if there is one.
					for b in range(1, ds.RasterCount + 1):
						if dc < 0 or dr < 0 or dc >= ds.RasterXSize or dr >= ds.RasterYSize:
							v = ''
						else:
							band = ds.GetRasterBand(b)
							v = float(band.ReadAsArray(dc, dr, 1, 1)[0,0])
							if isnan(v) or v == nd:
								keepRow = False
								break

						# Append the value to the row.
						csvrow.append(v)

				if keepRow:
					# Write the row to output.
					f.write(','.join(map(str, csvrow)) + '\n')


def get_values(filename, cls, columns):
	values = []
	data = []
	with open(filename, 'rU') as f:
		db = csv.reader(f)
		head = db.next()
		for row in db:
			if int(row[0]) == cls:
				for c in columns:
					values.append(float(row[c]))
				data.append(row)
	return np.array(values).reshape(-1, len(columns)), head, data


def cluster(job, k):

	values, head, data = get_values(job['filename'], job['cls'], job['columns'])
	kmeans = KMeans(n_clusters = k, random_state = 0)
	kmeans = kmeans.fit(values)
	return kmeans, values, head, data

def run_elbow(job, maxk=10):
	print 'Elbow'
	print 'Columns:', job['columns']
	print 'Class:', job['cls']
	inertia = 0.
	for k in range(1, maxk + 2):
		kmeans, values, head, data = cluster(job, k)
		print '- k:', k, '; inertia:', kmeans.inertia_, '; ratio:', kmeans.inertia_ / inertia
		inertia = kmeans.inertia_

def run(job):

	kmeans, values, head, data = cluster(job, job['k'])
	labels = kmeans.predict(values).reshape((-1, 1))
	#values = np.append(values, labels, axis = 1)
	#print len(values), len(data)
	with open(job['outfile'], 'w') as f:
		f.write(','.join(head + ['label']) + '\n') # Ingoring the centroids here; could keep them if desired.

		for i in range(len(data)):
			f.write(','.join(map(str, data[i] + list(labels[i]))) + '\n')
	
if __name__ == '__main__':

	#for job in jobs:
	#	run_elbow(job)

	#generate_all(rasters , clsraster, csvfile)
	for job in jobs:
		run(job)