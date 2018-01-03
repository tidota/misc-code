CFLAGS=-Wall -c -O2 -g

all: wastedris

wastedris: wastedris.o noncanonical.o game_core.o
	g++ -pthread -o $@ $^

game_core.o: game_core.cpp game_core.hpp format_macro.hpp
	g++ -std=c++11 $(CFLAGS) $<

noncanonical.o: noncanonical.cpp noncanonical.hpp
	g++ $(CFLAGS) $<

wastedris.o: wastedris.cpp game_core.hpp noncanonical.hpp
	g++ -std=c++11 $(CFLAGS) $<

clean:
	rm -rf *.o wastedris


