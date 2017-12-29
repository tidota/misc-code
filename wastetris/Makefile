
all: wastetris

wastetris: wastetris.o noncanonical.o
	g++ -o $@ $^

%.o: %.cpp %.hpp
	g++ -O2 -Wall -c $<

clean:
	rm -rf *.o wastetris


