#!/bin/bash

# Generates PDFs with Latex text from svgs.

#exit 0;

input=$1
output=$2

mkdir -p $output
rm -f $output/*.pdf
rm -f $output/*.pdf_tex

for f in $input/*.svg;
do
	echo "Exporting $f to $(basename $f .svg).pdf_tex"
	inkscape -D -z --file=$f --export-pdf=$output/$(basename $f .svg).pdf --export-latex
done

mkdir -p /tmp/plots
rm /tmp/plots/*
$input/point_clouds.py /tmp/plots

for f in /tmp/plots/*.svg;
do
	echo "Exporting $f to $(basename $f .svg).pdf_tex"
	inkscape -D -z --file=$f --export-pdf=$output/$(basename $f .svg).pdf --export-latex
done
