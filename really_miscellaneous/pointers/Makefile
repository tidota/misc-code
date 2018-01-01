EXEC=array1d2d func_p func_p_class

all: $(EXEC)

array1d2d: array1d2d.o
	gcc -o $@ $^

func_p: func_p.o
	gcc -o $@ $^

func_p_class: func_p_class.o
	g++ -o $@ $^

%.o: %.c
	gcc -c $<

%.o: %.cpp
	g++ -c $<

clean:
	rm -rf $(EXEC) *.o
