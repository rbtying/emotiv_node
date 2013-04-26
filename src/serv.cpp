#include <csignal>
#include <cstdlib>
#include <unistd.h>
#include <stdio.h>
#include <cstring>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <iostream>

#include "edk.h"
#include "edkErrorCode.h"
#include "EmoStateDLL.h"

#include <thread>
#include <mutex>
#include "request.h"
#include "die.h"
#include "data.h"

#include "JSON.h"

#define MAXPENDING 64

std::wstring s2w(std::string s) {
    std::wstringstream ss;
    ss << s.c_str();
    return ss.str();
}

std::string serialize_data(data_t *data) {
    std::lock_guard<std::mutex> lk(data->m);

    JSONObject root;

    JSONObject states;
    states[L"isBlink"] = new JSONValue(data->isBlink);
    states[L"isLeftWink"] = new JSONValue(data->isLeftWink);
    states[L"isRightWink"] = new JSONValue(data->isRightWink);
    states[L"isLookingLeft"] = new JSONValue(data->isLookingLeft);
    states[L"isLookingRight"] = new JSONValue(data->isLookingRight);

    JSONObject expressiv;

    expressiv[L"lowerFaceActionName"] = new JSONValue(s2w(data->lowerFaceActionName));
    expressiv[L"lowerFaceAction"] = new JSONValue((double) data->lowerFaceAction);
    expressiv[L"lowerFaceActionPower"] = new JSONValue(data->lowerFaceActionPower);
    expressiv[L"upperFaceActionName"] = new JSONValue(s2w(data->upperFaceActionName));
    expressiv[L"upperFaceAction"] = new JSONValue((double) data->upperFaceAction);
    expressiv[L"upperFaceActionPower"] = new JSONValue(data->upperFaceActionPower);

    root[L"time"] = new JSONValue(data->timestamp);
    root[L"battery"] =  new JSONValue(data->battery);
    root[L"wirelessSignal"] = new JSONValue((double)(data->wireless_signal));
    root[L"states"] = new JSONValue(states);
    root[L"expressiv"] = new JSONValue(expressiv);

    JSONArray array;
    for (int i = 0; i < data->cq.size(); i++) {
        array.push_back(new JSONValue((double) data->cq[i]));
    }
    root[L"contactQuality"] = new JSONValue(array);

    JSONValue *value = new JSONValue(root);
    std::wstring out = value->Stringify();


    std::string ss;
    ss.assign(out.begin(), out.end());

    delete value;

    return ss;
}

int serve(volatile bool *running, data_t *data, int port) {
    int pid;
    int servSock, clntSock;
    int children;

    struct sockaddr_in cli_addr;
    struct sockaddr_in serv_addr;

    if (port < 0 || port > 65336) {
        logerr("%d is an invalid port number\r\n", port);
    }

    if ((servSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        logerr("Could not open socket\r\n");
    }

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(port);

    while (*running && bind(servSock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        logerr("Could not bind to port %d, trying again in 1s\r\n", port);
        sleep(1);
    }
    logerr("Bound to port %d!\r\n", port);

    if (listen(servSock, MAXPENDING) < 0) {
        logerr("Listen failed\r\n");
    }

    int flags;

    if (-1 == (flags = fcntl(servSock, F_GETFL, 0))) {
        flags = 0;
    }
    fcntl(servSock, F_SETFL, flags | O_NONBLOCK);


    for (children = 1; *running; children++) {
        unsigned int length = sizeof(cli_addr);
        if ((clntSock = accept(servSock, (struct sockaddr *) &cli_addr, &length)) < 0) {
            if (clntSock < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    usleep(50);
                } else {
                    logerr("Accept failed\r\n");  
                    close(clntSock);
                }
            }
        } else {
            const char *content = "hi world";

            std::string json = serialize_data(data);

            servRequest(clntSock, cli_addr, json.c_str());
            close(clntSock);
        }
    }

    close(servSock);
    return 0;
}

