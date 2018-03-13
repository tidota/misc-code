// server.c
// 
// Server code
//
// references:
// https://www.geeksforgeeks.org/socket-programming-cc/
// https://www.binarytides.com/server-client-example-c-sockets-linux/
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#define PORT 8080

int main(int argc, char const *argv[])
{
    int server_fd;
    int client_socket;

    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    // socket file descriptor (it returns 0 if it fails)
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    // options to the socket (it returns non-zero if it fails)
    //setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    // settings of address and port
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
      
    // bind the socket and the address (it returns a negative value if it fails)
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));

    // set the socket to listening mode (it returns a negative value if it fails)
    listen(server_fd, 3);

    // wait for a request from a client (it returns a negative value if it fails)
    client_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);

    // read what it got (returns # of bytes it has read)
    recv( client_socket , buffer, 1024, 0);

    printf("%s\n",buffer );

    // write (similar to fwrite for file iO)
    send(client_socket , "hello from server", strlen("hello from server") , 0 );

    printf("Hello message sent\n");

    return 0;
}
