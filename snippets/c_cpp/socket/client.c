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
#define DEFAULT_PORT 8080
  
int main(int argc, char const *argv[])
{
    int sock = 0;
    int valread;
    struct sockaddr_in serv_addr;

    char *hello = "Hello from client";

    char buffer[1024] = {0};

    char add[32] = "127.0.0.1";
    int port_dest = DEFAULT_PORT;
    if (argc >= 2)
    {
        strcpy(add, argv[1]);
    }
    if (argc >= 3)
    {
        sscanf(argv[2], "%d", &port_dest);
    }

    printf("ADDRESS: %s\n", add);
    printf("PORT#: %d\n", port_dest);

    // it returns a negative value if it fails
    sock = socket(AF_INET, SOCK_STREAM, 0);
  
    // if the source port # is specified,
    if (argc >= 4)
    {
        int port_src;
        struct sockaddr_in clnt_addr;
        sscanf(argv[3], "%d", &port_src);
        clnt_addr.sin_family = AF_INET;
        clnt_addr.sin_addr.s_addr = INADDR_ANY;
        clnt_addr.sin_port = htons(port_src);
        clnt_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        if (bind(sock, (struct sockaddr*) &clnt_addr, sizeof(struct sockaddr_in)) == 0)
            printf("Specified the source port #: %d\n", port_src);
        else
            printf("Failed to specify the source port #: %d\n", port_src); 
    }

    // settings of server address and port
    serv_addr.sin_addr.s_addr = inet_addr(add);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port_dest);
      
    // connect to the server (it returns a negative value if it fails)
    int suc = connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    if (suc >= 0)
    {
        // write data
        send(sock , "Hello from client" , strlen("Hello from client") , 0 );

        printf("Message sent to the server: Hello from client\n");

        // read data
        recv( sock , buffer, 1024, 0);

        printf("Server said: %s\n",buffer );
    }
    else
    {
        printf("Connection failed\n");
    }

    return 0;
}
