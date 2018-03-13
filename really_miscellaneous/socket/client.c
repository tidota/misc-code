// client.c
//
// Simple client side code.
// It first sends a message and then receives a reply from the server.
// It does not handle any error.
//
// references:
// https://www.geeksforgeeks.org/socket-programming-cc/
// https://www.binarytides.com/server-client-example-c-sockets-linux/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/socket.h> // socket
#include <netinet/in.h> // sockaddr_in
#include <arpa/inet.h> // inet_addr
#define PORT 8080
  
int main(int argc, char const *argv[])
{
    int sock = 0;
    int valread;
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
    send(sock , "Hello from client" , strlen("Hello from client") , 0 );

    printf("Message sent to the server: Hello from client\n");

    // read data
    recv( sock , buffer, 1024, 0);

    printf("Server said: %s\n",buffer );

    return 0;
}
