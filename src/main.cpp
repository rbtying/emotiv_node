
// Data will be saved to "EmoStateLogger.csv" file

/* #include <QtCore/QCoreApplication> */
#include <iostream>
#include <csignal>
#include <cstdlib>

#include <thread>
#include "serv.h"
#include "Emotiv.h"

#include "data.h"

static volatile bool running = true;
static data_t data;

#define MAXPENDING 64

void servTask(int port) {
    serve(&running, &data, port);
}

void emotivTask() {
    emote(&running, &data);
}

void sighandler(int sig) {
    running = false;
    std::cout << "got here" << std::endl;
}

int main(int argc, char** argv) {
    std::signal(SIGTERM, &sighandler);
    std::signal(SIGINT, &sighandler);
    std::signal(SIGABRT, &sighandler);

    int port;

    if (argc >= 2) {
        port = atoi(argv[1]);
    } else {
        port = 8888;
    }

    data.lowerFaceActionName = "";
    data.upperFaceActionName = "";

    std::thread emotivThread = std::thread(emotivTask);
    std::thread servThread = std::thread(servTask, port);

    emotivThread.join();
    servThread.join();

    return 0;
}

