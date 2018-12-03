all: pthread std_thread

std_thread: std_thread.o
	g++ -pthread -o $@ $^

std_thread.o: std_thread.cpp
	g++ -std=c++11 -c $<

pthread: pthread.o
	g++ -pthread -o $@ $^

pthread.o: pthread.cpp
	g++ -c $<

clean:
	rm -rf pthread std_thread *.o

