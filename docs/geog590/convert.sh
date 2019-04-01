#!/bin/bash

mkdir -p tmp

for f in figures/*;
do
	base=$(basename $f)
	convert $f tmp/${base%.*}.pdf
	#inkscape -z -D --file=$f --export-pdf=tmp/${base%.*}.pdf --export-latex
done
