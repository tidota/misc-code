#include <iostream>
#include <iomanip>
#include <cstring>

#include <openssl/sha.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc == 2)
    {
        char str[] = "OriginalString";
        unsigned char hash1[SHA256_DIGEST_LENGTH];
        unsigned char hash2[SHA256_DIGEST_LENGTH];

        SHA256((const unsigned char*)str, strlen(str), hash1);
        SHA256((const unsigned char*)argv[1], strlen(argv[1]), hash2);

        cout << "string: " << str << endl;
        for(int i = 0; i < SHA256_DIGEST_LENGTH; i++)
        {
            cout << setw(2) << hex << (int)hash1[i] << ' ';
        }
        cout << endl;
        cout << "given from the terminal: " << argv[1] << endl;
        for(int i = 0; i < SHA256_DIGEST_LENGTH; i++)
        {
            cout << setw(2) << hex << (int)hash2[i] << ' ';
        }
        cout << endl;

        bool flag = true;
        for(int i = 0; i < SHA256_DIGEST_LENGTH; i++)
        {
            if(hash1[i] != hash2[i])
                flag = false;
        }
        cout << "matched?: ";
        if(flag)
            cout << "Yes" << endl;
        else
            cout << "No" << endl;
    }

    return 0;
}

