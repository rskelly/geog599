#!/bin/bash

# Generates PDFs with Latex text from svgs.

input=$1
output=$2

mkdir -p $2
rm -f $2/*.pdf
rm -f $2/*.pdf_tex

for f in $input/*.svg;
do
	echo "Exporting $f to $(basename $f .svg).pdf_tex"
	inkscape -D -z --file=$f --export-pdf=$output/$(basename $f .svg).pdf --export-latex
done