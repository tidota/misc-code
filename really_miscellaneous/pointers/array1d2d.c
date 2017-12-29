#include <stdio.h>

int main()
{

    /* ====================================== */
    /* dealing with a 1D array as a 2D array  */
    /* ====================================== */
    int array[9] = {0,1,2,3,4,5,6,7,8}; /* int [9] */
    int (*array2)[3]; /* int (*)[3]: a pointer to 2D array */
    int i, j;
    
    array2 = (int (*)[3])array;

    for(i = 0; i < 3; i++)
        for(j = 0; j < 3; j++)
            printf("array2[%d][%d] = %d\n",i,j,array2[i][j]);

    return 0;
}

