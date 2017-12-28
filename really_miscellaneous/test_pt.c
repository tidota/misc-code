#include <stdio.h>

int main()
{

    int array[9] = {0,1,2,3,4,5,6,7,8}; // int [9]
    int (*array2)[3]; // int (*)[3]
    
    array2 = (int (*)[3])array;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            printf("array2[%d][%d] = %d\n",i,j,array2[i][j]);

    return 0;
}

