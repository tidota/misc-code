all: client server

client: client.o
	gcc -o $@ $^

server: server.o
	gcc -o $@ $^

%.o: %.c
	gcc -c $^

clean:
	rm -rf client server *.o

