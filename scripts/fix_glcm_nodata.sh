#!/bin/bash -e

# Run from outside the lm_wetness folder, which contains the GLCM_* folders. A mask.tif file must be in the current dir.

for d in lm_wetness/GLCM_*;
do
	for f in $d/*.tif;
	do
		echo $f
		gdalbuildvrt -separate tmp.vrt $f mask.tif
		gdal_calc.py -A tmp.vrt --A 1 -B tmp.vrt --B 2 \
			--calc="-9999 + B * (9999 + numpy.nan_to_num(A))" \
			--outfile=tmp.tif --NoDataValue=-9999 --overwrite
		mv tmp.tif $f
	done
done