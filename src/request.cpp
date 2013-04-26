#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "die.h"
#include "request.h"

#define BUFSIZE 4096

const char *err501(int descriptor) {
    const char *response = "HTTP/1.0 501 Not Implemented\r\n\r\n<html>\r\n<body>\r\n<h1>501 Not Implemented</h1>\r\n</body>\r\n</html>\r\n";
    send(descriptor, response, strlen(response), 0);
    return "501 Not Implemented";
}

const char *err400(int descriptor) {
    const char *response = "HTTP/1.0 400 Bad Request\r\n\r\n<html>\r\n<body>\r\n<h1>400 Bad Request</h1>\r\n</body>\r\n</html>\r\n";
    send(descriptor, response, strlen(response), 0);
    return "400 Bad Request";
}

const char *err404(int descriptor) {
    const char *response = "HTTP/1.0 404 Not Found\r\n\r\n<html>\r\n<body>\r\n<h1>404 Not Found</h1>\r\n</body>\r\n</html>\r\n";
    send(descriptor, response, strlen(response), 0);
    return "404 Not Found";
}

void servRequest(int descriptor, struct sockaddr_in addr, const char *content) {
    char buf[BUFSIZE + 1];

    memset(buf, 0, BUFSIZE + 1);

    printf("%s", inet_ntoa(addr.sin_addr));

    const char *status = "200 OK";

    if (!read(descriptor, buf, BUFSIZE)) {
        status = err400(descriptor);
        printf(" Read Error ");
    } else {
        // read entire request
        char *dup = strdup(buf);
        int i;

        for (i = 0; i < strlen(dup); i++) {
            if (dup[i] == '\r' || dup[i] == '\n') {
                dup[i] = '\0';
            }
        }
        printf(" \"%s\" ", dup);
        free(dup);

        // process the request
        const char *token_separators = "\t \r\n";
        char *method = strtok(buf, token_separators);
        char *requestURI = strtok(NULL, token_separators);
        char *httpVersion = strtok(NULL, token_separators);

        if (method && requestURI && httpVersion) {
            if (strcmp(method, "GET")) {
                status = err501(descriptor);
            } else if (requestURI[0] != '/' || strstr(requestURI, "..")) {
                status = err400(descriptor);
            } else {
                const char *header = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n\r\n";
                // ignoring mimetype
                send(descriptor, header, strlen(header), 0);
                send(descriptor, content, strlen(content), 0);
                status = "200 OK";
            }
        } else {
            status = err400(descriptor);
        }
    }

    printf("%s\r\n", status);
}
