#include <stdio.h>

void foo(void*, void*);

int main()
{
    char str[] = "This is a string.";
    void *func_p = printf;

    foo(func_p, str);

    return 0;
}

void foo(void* f, void* s)
{
    ((void(*)(const char*, char*))f)("%s\n",(char*)s);
}

