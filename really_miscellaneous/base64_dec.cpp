#include <stdio.h>

#include <iostream>
#include <map>

using namespace std;


void makeTable(map<char,int>& tb)
{
    for(int i = 0; i < 26; i++)
        tb[(char)('A'+i)] = i;
    for(int i = 0; i < 26; i++)
        tb[(char)('a'+i)] = 26 + i;
    for(int i = 0; i < 10; i++)
        tb[(char)('0'+i)] = 52 + i;
    tb['+'] = 62;
    tb['/'] = 63;
}

void decrypt(FILE* rf, FILE* wf)
{
    map<char,int> table;
    makeTable(table);
    
    int curr_bit = 0;
    char ic;
    char oc;
    int val_curr = 0;
    int val_prev = 0;
    int val_prev2 = 0;
    int shift = 0;

    while((ic = fgetc(rf)) != EOF)
    {
        if(ic != '=')
        {
            oc = 0;
            if(curr_bit == 7)
            {
                oc = oc | (unsigned char)((val_prev2 << 7)&0xFF);
            }
            if(curr_bit != 0)
            {
                shift = 8 - curr_bit;
                oc = oc | (unsigned char)((val_prev << shift)&0xFF);
            }
            val_curr = table[ic];
            curr_bit = (curr_bit + 6) % 8;
            shift = curr_bit;
            oc = oc | (unsigned char)((val_curr >> shift) & 0xFF);

            if(curr_bit < 6)
            {
                fputc(oc,wf);
            }
            val_prev2 = val_prev;
            val_prev = val_curr;
        }
    }
}

int main(int argc, char** argv)
{
    if(argc == 2)
    {
        FILE* rf = fopen(argv[1],"r");
        
        if(rf != NULL)
        {
            FILE* wf = fopen("output.txt","w");
            if(wf != NULL)
            {
                decrypt(rf,wf);
                fclose(wf);
            }
            else
            {
                printf("file not found: output.txt\n");
            }
            fclose(rf);
        }
        else
        {
            printf("file not found: %s\n",argv[1]);
        }
    }
    else
    {
        printf("give a file name to decrypt.\n");
    }
}
