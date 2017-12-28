all: sha_test

sha_test: sha_test.o
	g++ -o $@ $^ -lssl -lcrypto

sha_test.o: sha_test.cpp
	g++ -c $^ -I.

clean:
	rm -rf sha_test *.o


