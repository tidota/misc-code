
all: wastetris

wastetris: wastetris.o noncanonical.o game_core.o
	g++ -o $@ $^

%.o: %.cpp %.hpp
	g++ -O2 -Wall -c $<

%.o: %.cpp
	g++ -O2 -Wall -c $<

clean:
	rm -rf *.o wastetris


