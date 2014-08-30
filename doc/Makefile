all: yaml_file_spec.pdf

%.pdf: %.tex
	pdflatex $<
	pdflatex $<
	pdflatex $<

clean:
	rm -rfv *~ *.log *.aux *.out *.backup

fullclean: clean
	rm -rfv *.pdf
