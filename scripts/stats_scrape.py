#!/usr/bin/env python3

import sys
import os
import csv

path = sys.argv[1]
prefix = sys.argv[2]

first = True
for f in [x for x in os.listdir(path) if x.startswith(prefix) and x.endswith('_stats.csv')]:
	with open(os.path.join(path, f), 'r') as ff:
		db = csv.reader(ff)
		head = next(db)
		if first:
			print(','.join(['filename'] + head))
			first = False
		for row in db:
			print(','.join([f] + row))

