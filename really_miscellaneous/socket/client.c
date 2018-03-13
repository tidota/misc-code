// client.c
//
// client side
//
// references:
// https://www.geeksforgeeks.org/socket-programming-cc/
// https://www.binarytides.com/server-client-example-c-sockets-linux/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h> 
#include <arpa/inet.h> // inet_addr
#define PORT 8080
  
int main(int argc, char const *argv[])
{
    int sock = 0;
    int valread;
    struct sockaddr_in address;
    struct sockaddr_in serv_addr;

    char *hello = "Hello from client";

    char buffer[1024] = {0};

    // it returns a negative value if it fails
    sock = socket(AF_INET, SOCK_STREAM, 0);
  
    // settings of server address and port
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
      
    // connect to the server (it returns a negative value if it fails)
    connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

    // write data
    send(sock , "hello from client" , strlen("hello from client") , 0 );

    printf("Hello message sent\n");

    // read data
    recv( sock , buffer, 1024, 0);

    printf("%s\n",buffer );

    return 0;
}
