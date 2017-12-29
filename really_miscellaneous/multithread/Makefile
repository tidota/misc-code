all: pthread

pthread: pthread.o
	g++ -pthread -o $@ $^

pthread.o: pthread.cpp
	g++ -c $<


clean:
	rm -rf pthread *.o

