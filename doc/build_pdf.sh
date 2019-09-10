#!/bin/bash

#for ext in `echo "aux bbl blg dvi log synctex.gz toc pdf "`; do rm "template".$ext &> /dev/null; done
pdflatex -synctex=1 -interaction=nonstopmode template.tex
pdflatex -synctex=1 -interaction=nonstopmode template.tex

if [ -f references.bib ]; then
	bibtex template.aux
	pdflatex -synctex=1 -interaction=nonstopmode template.tex
	pdflatex -synctex=1 -interaction=nonstopmode template.tex
fi
