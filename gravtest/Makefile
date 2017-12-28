all: gravtest

gravtest: gravtest.o
	g++ -o $@ $<

gravtest.o: gravtest.cpp
	g++ -ansi -pedantic-errors -Wall -c $<

clean:
	rm -rf gravtest *.o


