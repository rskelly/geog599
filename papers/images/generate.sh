#!/bin/bash

# Generates PDFs with Latex text from svgs.

rm -f *.pdf
rm -f *.pdf_tex

for f in *.svg;
do
	echo "Exporting $f to $(basename $f .svg).pdf_tex"
	inkscape -D -z --file=$f --export-pdf=$(basename $f .svg).pdf --export-latex
done