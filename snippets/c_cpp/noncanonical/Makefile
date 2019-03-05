all: noncan

noncan: noncanonical.o
	g++ -o $@ $<

%.o: %.cpp
	g++ -c $<

clean:
	rm -rf noncan *.o

