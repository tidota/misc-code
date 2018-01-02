
all: wastetris

wastetris: wastetris.o noncanonical.o game_core.o
	g++ -pthread -o $@ $^

game_core.o: game_core.cpp game_core.hpp format_macro.hpp
	g++ -std=c++11 -O2 -Wall -c $<

noncanonical.o: noncanonical.cpp noncanonical.hpp
	g++ -O2 -Wall -c $<

wastetris.o: wastetris.cpp game_core.hpp noncanonical.hpp
	g++ -std=c++11 -O2 -Wall -c $<

clean:
	rm -rf *.o wastetris


