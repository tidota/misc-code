EXEC=array1d2d func_p

all: $(EXEC)

array1d2d: array1d2d.o
	gcc -o $@ $^

func_p: func_p.o
	gcc -o $@ $^

%.o: %.c
	gcc -c $<

clean:
	rm -rf $(EXEC) *.o
