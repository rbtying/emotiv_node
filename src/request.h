#ifndef REQUEST_H_
#define REQUEST_H_

#include <arpa/inet.h>

void servRequest(int descriptor, struct sockaddr_in addr, const char *content);

#endif /* REQUEST_H_ */
