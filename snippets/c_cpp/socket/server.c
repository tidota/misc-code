// server.c
// 
// Simple server side code.
// It waits for a request from a client.
// Once it receives a request, it gets a message from the client and sends a reply.
// When it is done, it waits for another request and it will repeat for ever.
// It does not handle andy error.
//
// references:
// https://www.geeksforgeeks.org/socket-programming-cc/
// https://www.binarytides.com/server-client-example-c-sockets-linux/
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/socket.h> // socket
#include <netinet/in.h> // sockaddr_in
#include <arpa/inet.h> // inet_ntoa
#define DEFAULT_PORT 8080

int main(int argc, char const *argv[])
{
    int server_fd;
    int client_socket;

    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    int port = DEFAULT_PORT;
    if (argc == 2)
    {
        sscanf(argv[1], "%d", &port);
    }
    printf("PORT#: %d\n", port);

    // socket file descriptor (it returns 0 if it fails)
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    // options to the socket (it returns non-zero if it fails)
    //setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    // settings of address and port
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);
      
    // bind the socket and the address (it returns a negative value if it fails)
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));

    // set the socket to listening mode (it returns a negative value if it fails)
    listen(server_fd, 3);

    printf("Listening... (Ctrl+C to quit)\n");

    // each iteration handles one request from a client
    while(1)
    {
        // wait for a request from a client (it returns a negative value if it fails)
        client_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);

        printf("Request received\n");

        // read what it got (returns # of bytes it has read)
        recv(client_socket, buffer, 1024, 0);

        printf("Client(%s:%u) said: %s\n", (char *)inet_ntoa(address.sin_addr), address.sin_port, buffer);

        // write (similar to fwrite for file iO)
        send(client_socket, "Hello from server", strlen("Hello from server"), 0);

        printf("Message sent to the client: Hello from server\n");
    }

    return 0;
}
