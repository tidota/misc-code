EXEC=base64_dec

all: $(EXEC)

base64_dec: base64_dec.o
	g++ -o $@ $^

%.o: %.cpp
	g++ -c $^

clean:
	rm -rf $(EXEC) *.o

